#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import logging
import os
import queue
import select
import signal
import struct
import termios
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse


SERVICE_NAME = "sc-witmotion"
STATE_FILE_DEFAULT = "/var/lib/sc-witmotion/runtime.json"
ACC_RANGE_G = 16.0
GYRO_RANGE_DPS = 2000.0
ANGLE_RANGE_DEG = 180.0
MS2_PER_G = 9.80665
DEFAULT_STALE_MS = 750

FRAME_NAMES = {
    0x50: "time",
    0x51: "accel",
    0x52: "gyro",
    0x53: "angle",
    0x54: "mag",
    0x55: "port",
    0x56: "pressure",
    0x57: "gps",
    0x58: "velocity",
    0x59: "quaternion",
    0x5A: "gsa",
    0x5F: "register",
}

OUTPUT_CONTENT_BITS = (
    ("time", 0x01, "Time"),
    ("acc", 0x02, "Acceleration"),
    ("gyro", 0x04, "Gyro"),
    ("angle", 0x08, "Angle"),
    ("mag", 0x10, "Magnetometer"),
    ("port", 0x20, "Digital Port"),
    ("pressure", 0x40, "Pressure"),
    ("gps", 0x80, "GPS"),
    ("velocity", 0x100, "Velocity"),
    ("quaternion", 0x200, "Quaternion"),
    ("gsa", 0x400, "GSA"),
)

RATE_TO_CODE = {
    0.2: 0x01,
    0.5: 0x02,
    1.0: 0x03,
    2.0: 0x04,
    5.0: 0x05,
    10.0: 0x06,
    20.0: 0x07,
    50.0: 0x08,
    100.0: 0x09,
    125.0: 0x0A,
    200.0: 0x0B,
}
CODE_TO_RATE = {value: key for key, value in RATE_TO_CODE.items()}

BANDWIDTH_TO_CODE = {
    256: 0,
    184: 1,
    94: 2,
    44: 3,
    21: 4,
    10: 5,
    5: 6,
}
CODE_TO_BANDWIDTH = {value: key for key, value in BANDWIDTH_TO_CODE.items()}

ORIENTATION_LABELS = {
    0: "horizontal",
    1: "vertical",
}

AXIS_MODE_LABELS = {
    0: "9-axis",
    1: "6-axis",
}

KV_FIELDS = (
    "connected",
    "stale",
    "timestamp_ms",
    "age_ms",
    "sequence",
    "sample_rate_hz",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "accel_x_g",
    "accel_y_g",
    "accel_z_g",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "mag_x",
    "mag_y",
    "mag_z",
    "temperature_c",
    "version_raw",
    "device",
    "baud",
    "last_error",
)


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


def _env_str(name: str, default: str) -> str:
    return os.getenv(name, default)


def _coerce_nan(value: float | int | None) -> str:
    if value is None:
        return "NaN"
    return str(value)


def _checksum(frame: bytes) -> bool:
    return (sum(frame[:10]) & 0xFF) == frame[10]


def _baud_constant(baud: int) -> int:
    mapping: dict[int, int] = {
        2400: termios.B2400,
        4800: termios.B4800,
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
        460800: termios.B460800,
    }
    if hasattr(termios, "B921600"):
        mapping[921600] = getattr(termios, "B921600")
    if baud not in mapping:
        raise ValueError(f"Unsupported baud rate: {baud}")
    return mapping[baud]


def _supported_bauds() -> list[int]:
    baud_list = [2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800]
    if hasattr(termios, "B921600"):
        baud_list.append(921600)
    return baud_list


def _configure_serial(fd: int, baud: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] &= ~termios.CSIZE
    attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[2] &= ~termios.PARENB
    attrs[2] &= ~termios.CSTOPB
    if hasattr(termios, "CRTSCTS"):
        attrs[2] &= ~termios.CRTSCTS
    attrs[3] = 0
    speed = _baud_constant(baud)
    attrs[4] = speed
    attrs[5] = speed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcflush(fd, termios.TCIOFLUSH)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def _serial_write_bytes(register: int, value: int) -> bytes:
    return bytes((0xFF, 0xAA, register & 0xFF, value & 0xFF, (value >> 8) & 0xFF))


def _mask_to_content(mask: int | None) -> dict[str, bool]:
    if mask is None:
        mask = 0
    return {name: bool(mask & bit) for name, bit, _label in OUTPUT_CONTENT_BITS}


def _content_to_mask(content: dict[str, bool]) -> int:
    mask = 0
    for name, bit, _label in OUTPUT_CONTENT_BITS:
        if content.get(name):
            mask |= bit
    return mask


def _observed_mask(frame_counts: dict[str, int]) -> int:
    mapping = {
        "time": 0x01,
        "accel": 0x02,
        "gyro": 0x04,
        "angle": 0x08,
        "mag": 0x10,
        "port": 0x20,
        "pressure": 0x40,
        "gps": 0x80,
        "velocity": 0x100,
        "quaternion": 0x200,
        "gsa": 0x400,
    }
    mask = 0
    for frame_name, bit in mapping.items():
        if frame_counts.get(frame_name):
            mask |= bit
    return mask


def _nearest_supported_rate(rate_hz: float | None) -> float | None:
    if rate_hz is None or rate_hz <= 0:
        return None
    supported = list(RATE_TO_CODE.keys())
    return min(supported, key=lambda value: abs(value - rate_hz))


def _decode_gps_coordinate(raw: int) -> float:
    return (raw // 10000000.0 * 100.0) + ((raw % 10000000) / 10000000.0)


def _read_index_html() -> bytes:
    html_path = Path(__file__).with_name("index.html")
    return html_path.read_text(encoding="utf-8").encode("utf-8")


def _path_kind(path: str) -> str:
    if path.startswith("/dev/serial/by-path/"):
        return "by-path"
    if path.startswith("/dev/serial/by-id/"):
        return "by-id"
    if path.startswith("/dev/ttyUSB"):
        return "ttyUSB"
    if path.startswith("/dev/ttyACM"):
        return "ttyACM"
    return "other"


def _device_sort_key(path: str) -> tuple[int, str]:
    priority = {
        "by-path": 0,
        "by-id": 1,
        "ttyUSB": 2,
        "ttyACM": 3,
        "other": 4,
    }
    return (priority.get(_path_kind(path), 9), path)


def _scan_serial_devices(configured_path: str = "") -> list[dict[str, Any]]:
    alias_paths: list[str] = []
    for pattern in (
        "/dev/serial/by-path/*",
        "/dev/serial/by-id/*",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
    ):
        for path in sorted(Path("/").glob(pattern.lstrip("/"))):
            alias = str(path)
            if alias not in alias_paths:
                alias_paths.append(alias)

    grouped: dict[str, list[str]] = {}
    for alias in alias_paths:
        resolved = os.path.realpath(alias)
        grouped.setdefault(resolved, []).append(alias)

    entries: list[dict[str, Any]] = []
    for resolved_path, aliases in sorted(grouped.items()):
        ordered_aliases = sorted(aliases, key=_device_sort_key)
        persistent_path = ordered_aliases[0]
        label = persistent_path
        extra_aliases = [alias for alias in ordered_aliases if alias != persistent_path]
        if extra_aliases:
            label = f"{persistent_path} ({', '.join(extra_aliases)})"
        entries.append(
            {
                "path": persistent_path,
                "resolved_path": resolved_path,
                "aliases": ordered_aliases,
                "available": True,
                "kind": _path_kind(persistent_path),
                "label": label,
            }
        )

    if configured_path and all(entry["path"] != configured_path for entry in entries):
        entries.insert(
            0,
            {
                "path": configured_path,
                "resolved_path": "",
                "aliases": [configured_path],
                "available": False,
                "kind": _path_kind(configured_path),
                "label": f"{configured_path} (missing)",
            },
        )

    return entries


def _canonical_device_path(selected_path: str) -> str:
    if not selected_path:
        return ""
    if not os.path.exists(selected_path):
        return selected_path

    resolved = os.path.realpath(selected_path)
    for entry in _scan_serial_devices():
        if entry["resolved_path"] == resolved:
            return str(entry["path"])
    return selected_path


def _build_meta_payload() -> dict[str, Any]:
    return {
        "baud_options": [{"value": baud, "label": str(baud)} for baud in _supported_bauds()],
        "rate_options": [{"value": rate, "label": f"{rate:g} Hz"} for rate in RATE_TO_CODE],
        "bandwidth_options": [{"value": bandwidth, "label": f"{bandwidth} Hz"} for bandwidth in BANDWIDTH_TO_CODE],
        "orientation_options": [{"value": value, "label": label.title()} for value, label in ORIENTATION_LABELS.items()],
        "axis_mode_options": [{"value": value, "label": label} for value, label in AXIS_MODE_LABELS.items()],
        "output_options": [{"key": key, "label": label, "bit": bit} for key, bit, label in OUTPUT_CONTENT_BITS],
    }


@dataclass
class ControllerSettings:
    selected_device_path: str = ""
    host_baud: int = 115200
    reconnect_ms: int = 2000
    sensor_baud: int | None = None
    output_rate_hz: float | None = None
    bandwidth_hz: int | None = None
    orientation: int | None = None
    axis_mode: int | None = None
    output_content_mask: int | None = None

    def to_persisted_dict(self) -> dict[str, Any]:
        return {
            "selected_device_path": self.selected_device_path,
            "host_baud": self.host_baud,
            "reconnect_ms": self.reconnect_ms,
            "sensor_baud": self.sensor_baud,
            "output_rate_hz": self.output_rate_hz,
            "bandwidth_hz": self.bandwidth_hz,
            "orientation": self.orientation,
            "axis_mode": self.axis_mode,
            "output_content_mask": self.output_content_mask,
        }

    def to_runtime_dict(self) -> dict[str, Any]:
        return {
            "selected_device_path": self.selected_device_path,
            "host_baud": self.host_baud,
            "reconnect_ms": self.reconnect_ms,
            "sensor": {
                "sensor_baud": self.sensor_baud,
                "output_rate_hz": self.output_rate_hz,
                "bandwidth_hz": self.bandwidth_hz,
                "orientation": self.orientation,
                "axis_mode": self.axis_mode,
                "output_content_mask": self.output_content_mask,
                "output_content": _mask_to_content(self.output_content_mask),
            },
        }


class RuntimeConfigStore:
    def __init__(self, path: str, defaults: ControllerSettings) -> None:
        self._path = Path(path)
        self._defaults = defaults

    def load(self) -> ControllerSettings:
        settings = ControllerSettings(
            selected_device_path=self._defaults.selected_device_path,
            host_baud=self._defaults.host_baud,
            reconnect_ms=self._defaults.reconnect_ms,
            sensor_baud=self._defaults.sensor_baud,
            output_rate_hz=self._defaults.output_rate_hz,
            bandwidth_hz=self._defaults.bandwidth_hz,
            orientation=self._defaults.orientation,
            axis_mode=self._defaults.axis_mode,
            output_content_mask=self._defaults.output_content_mask,
        )

        if not self._path.exists():
            return settings

        try:
            persisted = json.loads(self._path.read_text(encoding="utf-8"))
        except Exception as exc:
            logging.warning("Failed to read runtime settings: %s", exc)
            return settings

        if not isinstance(persisted, dict):
            return settings

        settings.selected_device_path = str(persisted.get("selected_device_path") or settings.selected_device_path)
        settings.host_baud = int(persisted.get("host_baud") or settings.host_baud)
        settings.reconnect_ms = int(persisted.get("reconnect_ms") or settings.reconnect_ms)
        settings.sensor_baud = _nullable_int(persisted.get("sensor_baud"))
        settings.output_rate_hz = _nullable_float(persisted.get("output_rate_hz"))
        settings.bandwidth_hz = _nullable_int(persisted.get("bandwidth_hz"))
        settings.orientation = _nullable_int(persisted.get("orientation"))
        settings.axis_mode = _nullable_int(persisted.get("axis_mode"))
        settings.output_content_mask = _nullable_int(persisted.get("output_content_mask"))
        return settings

    def save(self, settings: ControllerSettings) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = self._path.with_suffix(".tmp")
        temp_path.write_text(json.dumps(settings.to_persisted_dict(), indent=2, sort_keys=True), encoding="utf-8")
        temp_path.replace(self._path)


def _nullable_int(value: Any) -> int | None:
    if value in (None, ""):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _nullable_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


@dataclass
class LatestData:
    roll_deg: float | None = None
    pitch_deg: float | None = None
    yaw_deg: float | None = None
    accel_x_g: float | None = None
    accel_y_g: float | None = None
    accel_z_g: float | None = None
    accel_x_mps2: float | None = None
    accel_y_mps2: float | None = None
    accel_z_mps2: float | None = None
    gyro_x_dps: float | None = None
    gyro_y_dps: float | None = None
    gyro_z_dps: float | None = None
    mag_x: int | None = None
    mag_y: int | None = None
    mag_z: int | None = None
    temperature_c: float | None = None
    version_raw: int | None = None
    quat_w: float | None = None
    quat_x: float | None = None
    quat_y: float | None = None
    quat_z: float | None = None
    latitude_deg: float | None = None
    longitude_deg: float | None = None
    gps_altitude_m: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "roll_deg": self.roll_deg,
            "pitch_deg": self.pitch_deg,
            "yaw_deg": self.yaw_deg,
            "accel_x_g": self.accel_x_g,
            "accel_y_g": self.accel_y_g,
            "accel_z_g": self.accel_z_g,
            "accel_x_mps2": self.accel_x_mps2,
            "accel_y_mps2": self.accel_y_mps2,
            "accel_z_mps2": self.accel_z_mps2,
            "gyro_x_dps": self.gyro_x_dps,
            "gyro_y_dps": self.gyro_y_dps,
            "gyro_z_dps": self.gyro_z_dps,
            "mag_x": self.mag_x,
            "mag_y": self.mag_y,
            "mag_z": self.mag_z,
            "temperature_c": self.temperature_c,
            "version_raw": self.version_raw,
            "quat_w": self.quat_w,
            "quat_x": self.quat_x,
            "quat_y": self.quat_y,
            "quat_z": self.quat_z,
            "latitude_deg": self.latitude_deg,
            "longitude_deg": self.longitude_deg,
            "gps_altitude_m": self.gps_altitude_m,
        }


@dataclass
class RuntimeState:
    configured_device_path: str = ""
    configured_device_present: bool = False
    device_path: str = ""
    resolved_device_path: str = ""
    baud: int = 115200
    connected: bool = False
    last_error: str = ""
    last_frame_ms: int = 0
    bytes_received: int = 0
    frames_received: int = 0
    checksum_errors: int = 0
    dropped_bytes: int = 0
    last_frame_name: str = ""
    frame_counts: dict[str, int] = field(default_factory=dict)
    latest: LatestData = field(default_factory=LatestData)
    available_devices: list[dict[str, Any]] = field(default_factory=list)
    settings: dict[str, Any] = field(default_factory=dict)


class StateStore:
    def __init__(self, base_stale_ms: int) -> None:
        self._lock = threading.Lock()
        self._state = RuntimeState()
        self._base_stale_ms = base_stale_ms
        self._frame_times_ms: deque[int] = deque(maxlen=4096)

    def update_serial_binding(
        self,
        configured_device_path: str,
        configured_device_present: bool,
        device_path: str,
        resolved_device_path: str,
        baud: int,
    ) -> None:
        with self._lock:
            self._state.configured_device_path = configured_device_path
            self._state.configured_device_present = configured_device_present
            self._state.device_path = device_path
            self._state.resolved_device_path = resolved_device_path
            self._state.baud = baud

    def update_devices(self, devices: list[dict[str, Any]]) -> None:
        with self._lock:
            self._state.available_devices = devices

    def update_settings(self, settings_payload: dict[str, Any]) -> None:
        with self._lock:
            self._state.settings = settings_payload

    def set_connected(self, connected: bool, error: str = "") -> None:
        with self._lock:
            self._state.connected = connected
            self._state.last_error = error.replace("\n", " ").strip()

    def note_error(self, error: str) -> None:
        with self._lock:
            self._state.last_error = error.replace("\n", " ").strip()
            self._state.connected = False

    def update_frames(self, chunk_len: int, frames: list[bytes], dropped: int, checksum_errors: int) -> None:
        now_ms = int(time.time() * 1000)
        with self._lock:
            self._state.bytes_received += chunk_len
            self._state.dropped_bytes += dropped
            self._state.checksum_errors += checksum_errors
            for frame in frames:
                frame_name, update = decode_frame(frame)
                self._state.frames_received += 1
                self._state.last_frame_ms = now_ms
                self._state.last_frame_name = frame_name
                self._state.frame_counts[frame_name] = self._state.frame_counts.get(frame_name, 0) + 1
                self._frame_times_ms.append(now_ms)
                apply_update(self._state.latest, update)
            if frames:
                self._state.connected = True
                self._state.last_error = ""

    def _sample_rate_locked(self, now_ms: int) -> float:
        recent = [frame_time for frame_time in self._frame_times_ms if now_ms - frame_time <= 1000]
        recent_rate = float(len(recent))
        rolling_rate = 0.0
        if len(self._frame_times_ms) >= 2:
            sample = list(self._frame_times_ms)[-min(len(self._frame_times_ms), 8):]
            span_ms = sample[-1] - sample[0]
            if span_ms > 0:
                rolling_rate = ((len(sample) - 1) * 1000.0) / span_ms
        return recent_rate if recent_rate > 0 else rolling_rate

    def snapshot(self) -> dict[str, Any]:
        now_ms = int(time.time() * 1000)
        with self._lock:
            age_ms = (now_ms - self._state.last_frame_ms) if self._state.last_frame_ms else 1_000_000_000
            sample_rate_hz = self._sample_rate_locked(now_ms)
            configured_sensor = dict(self._state.settings.get("sensor", {}))
            configured_rate_hz = _nullable_float(configured_sensor.get("output_rate_hz"))
            effective_rate_hz = configured_rate_hz or _nearest_supported_rate(sample_rate_hz)
            stale_threshold_ms = self._base_stale_ms
            if effective_rate_hz:
                stale_threshold_ms = max(self._base_stale_ms, int((1000.0 / effective_rate_hz) * 3.5))
            disconnect_threshold_ms = max(stale_threshold_ms * 2, 1500)
            stale = age_ms > stale_threshold_ms

            effective_mask = _nullable_int(configured_sensor.get("output_content_mask"))
            if effective_mask is None:
                effective_mask = _observed_mask(self._state.frame_counts)
            effective_sensor = {
                "sensor_baud": _nullable_int(configured_sensor.get("sensor_baud")) or self._state.baud,
                "output_rate_hz": effective_rate_hz,
                "bandwidth_hz": _nullable_int(configured_sensor.get("bandwidth_hz")),
                "orientation": _nullable_int(configured_sensor.get("orientation")),
                "axis_mode": _nullable_int(configured_sensor.get("axis_mode")),
                "output_content_mask": effective_mask,
                "output_content": _mask_to_content(effective_mask),
            }

            return {
                "service": SERVICE_NAME,
                "connected": self._state.connected,
                "stale": stale,
                "device_path": self._state.device_path,
                "resolved_device_path": self._state.resolved_device_path,
                "configured_device_path": self._state.configured_device_path,
                "configured_device_present": self._state.configured_device_present,
                "baud": self._state.baud,
                "timestamp_ms": self._state.last_frame_ms,
                "age_ms": age_ms,
                "stale_threshold_ms": stale_threshold_ms,
                "disconnect_threshold_ms": disconnect_threshold_ms,
                "frames_received": self._state.frames_received,
                "bytes_received": self._state.bytes_received,
                "checksum_errors": self._state.checksum_errors,
                "dropped_bytes": self._state.dropped_bytes,
                "last_frame_name": self._state.last_frame_name,
                "frame_counts": dict(sorted(self._state.frame_counts.items())),
                "sample_rate_hz": sample_rate_hz,
                "last_error": self._state.last_error,
                "latest": self._state.latest.to_dict(),
                "available_devices": list(self._state.available_devices),
                "settings": dict(self._state.settings),
                "effective_settings": {
                    "selected_device_path": self._state.settings.get("selected_device_path", ""),
                    "host_baud": self._state.settings.get("host_baud", self._state.baud),
                    "reconnect_ms": self._state.settings.get("reconnect_ms", 2000),
                    "sensor": effective_sensor,
                },
                "meta": _build_meta_payload(),
            }


def apply_update(latest: LatestData, update: dict[str, Any]) -> None:
    for key, value in update.items():
        setattr(latest, key, value)


def decode_frame(frame: bytes) -> tuple[str, dict[str, Any]]:
    frame_id = frame[1]
    name = FRAME_NAMES.get(frame_id, f"0x{frame_id:02x}")
    payload = frame[2:10]
    update: dict[str, Any] = {}

    if frame_id == 0x51:
        ax, ay, az, temp = struct.unpack("<hhhh", payload)
        update = {
            "accel_x_g": ax / 32768.0 * ACC_RANGE_G,
            "accel_y_g": ay / 32768.0 * ACC_RANGE_G,
            "accel_z_g": az / 32768.0 * ACC_RANGE_G,
            "accel_x_mps2": ax / 32768.0 * ACC_RANGE_G * MS2_PER_G,
            "accel_y_mps2": ay / 32768.0 * ACC_RANGE_G * MS2_PER_G,
            "accel_z_mps2": az / 32768.0 * ACC_RANGE_G * MS2_PER_G,
            "temperature_c": temp / 100.0,
        }
    elif frame_id == 0x52:
        gx, gy, gz, _ignored = struct.unpack("<hhhh", payload)
        update = {
            "gyro_x_dps": gx / 32768.0 * GYRO_RANGE_DPS,
            "gyro_y_dps": gy / 32768.0 * GYRO_RANGE_DPS,
            "gyro_z_dps": gz / 32768.0 * GYRO_RANGE_DPS,
        }
    elif frame_id == 0x53:
        roll, pitch, yaw, version = struct.unpack("<hhhh", payload)
        update = {
            "roll_deg": roll / 32768.0 * ANGLE_RANGE_DEG,
            "pitch_deg": pitch / 32768.0 * ANGLE_RANGE_DEG,
            "yaw_deg": yaw / 32768.0 * ANGLE_RANGE_DEG,
            "version_raw": version,
        }
    elif frame_id == 0x54:
        mx, my, mz, _ignored = struct.unpack("<hhhh", payload)
        update = {
            "mag_x": mx,
            "mag_y": my,
            "mag_z": mz,
        }
    elif frame_id == 0x57:
        longitude_raw, latitude_raw = struct.unpack("<ii", payload)
        update = {
            "longitude_deg": _decode_gps_coordinate(longitude_raw),
            "latitude_deg": _decode_gps_coordinate(latitude_raw),
        }
    elif frame_id == 0x58:
        altitude_dm, _reserved0, _reserved1, _reserved2 = struct.unpack("<hhhh", payload)
        update = {
            "gps_altitude_m": altitude_dm / 10.0,
        }
    elif frame_id == 0x59:
        q0, q1, q2, q3 = struct.unpack("<hhhh", payload)
        update = {
            "quat_w": q0 / 32768.0,
            "quat_x": q1 / 32768.0,
            "quat_y": q2 / 32768.0,
            "quat_z": q3 / 32768.0,
        }

    return name, update


class WitFrameBuffer:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> tuple[list[bytes], int, int]:
        self._buffer.extend(chunk)
        frames: list[bytes] = []
        dropped = 0
        checksum_errors = 0

        while True:
            start = self._buffer.find(0x55)
            if start < 0:
                dropped += len(self._buffer)
                self._buffer.clear()
                break
            if start > 0:
                dropped += start
                del self._buffer[:start]
            if len(self._buffer) < 11:
                break
            candidate = bytes(self._buffer[:11])
            if _checksum(candidate):
                frames.append(candidate)
                del self._buffer[:11]
            else:
                checksum_errors += 1
                del self._buffer[0]

        return frames, dropped, checksum_errors


@dataclass
class ControlRequest:
    action: str
    payload: dict[str, Any]
    event: threading.Event = field(default_factory=threading.Event)
    response: dict[str, Any] | None = None
    error: str | None = None


class SerialDisconnectError(RuntimeError):
    pass


class SerialController(threading.Thread):
    def __init__(
        self,
        state: StateStore,
        config_store: RuntimeConfigStore,
        settings: ControllerSettings,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(name="witmotion-serial", daemon=True)
        self._state = state
        self._config_store = config_store
        self._settings = settings
        self._stop_event = stop_event
        self._requests: queue.Queue[ControlRequest] = queue.Queue()
        self._fd: int | None = None
        self._current_device_path = ""
        self._current_resolved_path = ""
        self._reopen_requested = False
        self._last_rx_monotonic = 0.0
        self._state.update_settings(self._settings.to_runtime_dict())

    def submit(self, action: str, payload: dict[str, Any], timeout_s: float = 10.0) -> dict[str, Any]:
        request = ControlRequest(action=action, payload=payload)
        self._requests.put(request)
        if not request.event.wait(timeout_s):
            raise TimeoutError(f"Timed out waiting for {action}")
        if request.error:
            raise RuntimeError(request.error)
        return request.response or {}

    def run(self) -> None:
        parser = WitFrameBuffer()

        while not self._stop_event.is_set():
            device_entries = _scan_serial_devices(self._settings.selected_device_path)
            self._state.update_devices(device_entries)

            target_path = ""
            configured_present = False
            if self._settings.selected_device_path:
                configured_present = any(
                    entry["path"] == self._settings.selected_device_path and entry["available"] for entry in device_entries
                )
                if configured_present:
                    target_path = self._settings.selected_device_path
                else:
                    self._state.update_serial_binding(
                        self._settings.selected_device_path,
                        False,
                        "",
                        "",
                        self._settings.host_baud,
                    )
                    self._state.note_error("Configured serial device is not present")
                    self._wait_with_requests(self._settings.reconnect_ms / 1000.0)
                    continue
            else:
                for entry in device_entries:
                    if entry["available"]:
                        target_path = str(entry["path"])
                        break
                configured_present = bool(target_path)
                if not target_path:
                    self._state.update_serial_binding("", False, "", "", self._settings.host_baud)
                    self._state.note_error("No serial devices found")
                    self._wait_with_requests(self._settings.reconnect_ms / 1000.0)
                    continue

            self._current_device_path = target_path
            self._current_resolved_path = os.path.realpath(target_path)
            self._state.update_serial_binding(
                self._settings.selected_device_path,
                configured_present,
                self._current_device_path,
                self._current_resolved_path,
                self._settings.host_baud,
            )

            try:
                self._fd = os.open(self._current_device_path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                _configure_serial(self._fd, self._settings.host_baud)
                parser = WitFrameBuffer()
                self._last_rx_monotonic = time.monotonic()
                self._reopen_requested = False
                self._state.set_connected(True)
                logging.info("Opened %s at %d baud", self._current_device_path, self._settings.host_baud)

                while not self._stop_event.is_set():
                    self._process_requests()
                    if self._reopen_requested:
                        raise SerialDisconnectError("Reconnecting with updated settings")

                    readable, _, _ = select.select([self._fd], [], [], 0.2)
                    if readable:
                        chunk = os.read(self._fd, 512)
                        if not chunk:
                            raise SerialDisconnectError("Serial device returned EOF")
                        self._last_rx_monotonic = time.monotonic()
                        frames, dropped, checksum_errors = parser.feed(chunk)
                        self._state.update_frames(len(chunk), frames, dropped, checksum_errors)
                    else:
                        if self._disconnect_condition():
                            raise SerialDisconnectError(self._disconnect_reason())
            except Exception as exc:
                self._state.note_error(str(exc))
                logging.warning("Serial controller reconnecting after error: %s", exc)
            finally:
                if self._fd is not None:
                    try:
                        os.close(self._fd)
                    except OSError:
                        pass
                self._fd = None
                self._state.set_connected(False, self._state.snapshot()["last_error"])

            self._wait_with_requests(self._settings.reconnect_ms / 1000.0)

    def _wait_with_requests(self, seconds: float) -> None:
        end_time = time.monotonic() + max(seconds, 0.1)
        while not self._stop_event.is_set() and time.monotonic() < end_time:
            self._process_requests(timeout=0.1)

    def _disconnect_condition(self) -> bool:
        snapshot = self._state.snapshot()
        if self._settings.selected_device_path and not os.path.exists(self._settings.selected_device_path):
            return True
        if self._current_device_path and not os.path.exists(self._current_device_path):
            return True
        elapsed_ms = int((time.monotonic() - self._last_rx_monotonic) * 1000.0)
        return elapsed_ms > int(snapshot["disconnect_threshold_ms"])

    def _disconnect_reason(self) -> str:
        snapshot = self._state.snapshot()
        if self._settings.selected_device_path and not os.path.exists(self._settings.selected_device_path):
            return "Configured serial device disconnected"
        if self._current_device_path and not os.path.exists(self._current_device_path):
            return "Serial device disconnected"
        elapsed_ms = int((time.monotonic() - self._last_rx_monotonic) * 1000.0)
        return f"No serial data for {elapsed_ms} ms"

    def _process_requests(self, timeout: float = 0.0) -> None:
        while not self._stop_event.is_set():
            try:
                request = self._requests.get(timeout=timeout)
            except queue.Empty:
                return
            timeout = 0.0
            try:
                request.response = self._handle_request(request.action, request.payload)
            except Exception as exc:
                request.error = str(exc)
            finally:
                request.event.set()

    def _handle_request(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        if action == "apply_usb_settings":
            return self._apply_usb_settings(payload)
        if action == "apply_sensor_settings":
            return self._apply_sensor_settings(payload)
        if action == "sensor_action":
            return self._run_sensor_action(payload)
        raise RuntimeError(f"Unsupported action: {action}")

    def _apply_usb_settings(self, payload: dict[str, Any]) -> dict[str, Any]:
        selected_device = _canonical_device_path(str(payload.get("device_path") or ""))
        host_baud = int(payload.get("host_baud") or self._settings.host_baud)
        reconnect_ms = int(payload.get("reconnect_ms") or self._settings.reconnect_ms)
        _baud_constant(host_baud)

        self._settings.selected_device_path = selected_device
        self._settings.host_baud = host_baud
        self._settings.reconnect_ms = max(reconnect_ms, 250)
        self._config_store.save(self._settings)
        self._state.update_settings(self._settings.to_runtime_dict())
        self._state.note_error("USB settings updated; reconnecting")
        self._reopen_requested = True

        return {
            "ok": True,
            "message": "USB settings saved",
            "snapshot": self._state.snapshot(),
        }

    def _apply_sensor_settings(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._fd is None:
            raise RuntimeError("Sensor is not currently connected")

        output_content = payload.get("output_content")
        output_mask = _content_to_mask(output_content) if isinstance(output_content, dict) else None
        output_rate_hz = _nullable_float(payload.get("output_rate_hz"))
        bandwidth_hz = _nullable_int(payload.get("bandwidth_hz"))
        orientation = _nullable_int(payload.get("orientation"))
        axis_mode = _nullable_int(payload.get("axis_mode"))
        sensor_baud = _nullable_int(payload.get("sensor_baud"))

        write_queue: list[tuple[int, int]] = []

        if output_mask is not None:
            write_queue.append((0x02, output_mask))
        if output_rate_hz is not None:
            if output_rate_hz not in RATE_TO_CODE:
                raise RuntimeError("Unsupported output rate")
            write_queue.append((0x03, RATE_TO_CODE[output_rate_hz]))
        if sensor_baud is not None:
            if sensor_baud not in BAUD_TO_CODE:
                raise RuntimeError("Unsupported sensor baud")
            write_queue.append((0x04, BAUD_TO_CODE[sensor_baud]))
        if bandwidth_hz is not None:
            if bandwidth_hz not in BANDWIDTH_TO_CODE:
                raise RuntimeError("Unsupported bandwidth")
            write_queue.append((0x1F, BANDWIDTH_TO_CODE[bandwidth_hz]))
        if orientation is not None:
            if orientation not in ORIENTATION_LABELS:
                raise RuntimeError("Unsupported orientation")
            write_queue.append((0x23, orientation))
        if axis_mode is not None:
            if axis_mode not in AXIS_MODE_LABELS:
                raise RuntimeError("Unsupported axis mode")
            write_queue.append((0x24, axis_mode))

        if not write_queue:
            raise RuntimeError("No sensor settings provided")

        self._unlock_sensor()
        for register, value in write_queue:
            self._write_register(register, value)
            time.sleep(0.05)
        self._save_sensor()

        if output_mask is not None:
            self._settings.output_content_mask = output_mask
        if output_rate_hz is not None:
            self._settings.output_rate_hz = output_rate_hz
        if bandwidth_hz is not None:
            self._settings.bandwidth_hz = bandwidth_hz
        if orientation is not None:
            self._settings.orientation = orientation
        if axis_mode is not None:
            self._settings.axis_mode = axis_mode
        if sensor_baud is not None:
            self._settings.sensor_baud = sensor_baud
            self._settings.host_baud = sensor_baud

        self._config_store.save(self._settings)
        self._state.update_settings(self._settings.to_runtime_dict())

        message = "Sensor settings applied"
        if sensor_baud is not None:
            self._state.note_error("Sensor baud changed; reconnecting")
            self._reopen_requested = True
            message = "Sensor settings applied; reconnecting at new baud"

        return {
            "ok": True,
            "message": message,
            "snapshot": self._state.snapshot(),
        }

    def _run_sensor_action(self, payload: dict[str, Any]) -> dict[str, Any]:
        action = str(payload.get("action") or "")
        if action == "rescan":
            self._state.note_error("Rescanning serial devices")
            self._reopen_requested = True
            return {"ok": True, "message": "Rescanning serial devices", "snapshot": self._state.snapshot()}

        if self._fd is None:
            raise RuntimeError("Sensor is not currently connected")

        if action == "save_parameters":
            self._save_sensor()
            message = "Sensor parameters saved"
        elif action == "set_reference_angle":
            self._unlock_sensor()
            self._write_register(0x01, 0x08)
            message = "Reference angle command sent"
        elif action == "start_accel_calibration":
            self._unlock_sensor()
            self._write_register(0x01, 0x01)
            message = "Acceleration calibration started"
        elif action == "stop_accel_calibration":
            self._unlock_sensor()
            self._write_register(0x01, 0x00)
            message = "Acceleration calibration stopped"
        elif action == "start_mag_calibration":
            self._unlock_sensor()
            self._write_register(0x01, 0x07)
            message = "Mag calibration started"
        elif action == "stop_mag_calibration":
            self._unlock_sensor()
            self._write_register(0x01, 0x00)
            time.sleep(0.05)
            self._save_sensor()
            message = "Mag calibration stopped and saved"
        else:
            raise RuntimeError("Unsupported action")

        return {
            "ok": True,
            "message": message,
            "snapshot": self._state.snapshot(),
        }

    def _write_all(self, data: bytes) -> None:
        if self._fd is None:
            raise RuntimeError("Serial device is not open")
        offset = 0
        while offset < len(data):
            written = os.write(self._fd, data[offset:])
            if written <= 0:
                raise RuntimeError("Failed to write to serial device")
            offset += written

    def _write_register(self, register: int, value: int) -> None:
        self._write_all(_serial_write_bytes(register, value))

    def _unlock_sensor(self) -> None:
        self._write_register(0x69, 0xB588)
        time.sleep(0.05)

    def _save_sensor(self) -> None:
        self._write_register(0x00, 0x0000)
        time.sleep(0.05)


BAUD_TO_CODE = {
    4800: 1,
    9600: 2,
    19200: 3,
    38400: 4,
    57600: 5,
    115200: 6,
    230400: 7,
    460800: 8,
    921600: 9,
}


class WitRequestHandler(BaseHTTPRequestHandler):
    server_version = "sc-witmotion/0.2"

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        snapshot = self.server.state.snapshot()  # type: ignore[attr-defined]

        if path == "/":
            self._send_bytes(HTTPStatus.OK, "text/html; charset=utf-8", _read_index_html())
            return

        if path == "/healthz":
            status = HTTPStatus.OK if snapshot["connected"] and not snapshot["stale"] else HTTPStatus.SERVICE_UNAVAILABLE
            self._send_json(status, snapshot)
            return

        if path == "/api/v1/status":
            self._send_json(HTTPStatus.OK, snapshot)
            return

        if path == "/api/v1/imu.kv":
            self._send_bytes(HTTPStatus.OK, "text/plain; charset=utf-8", build_kv(snapshot).encode("utf-8"))
            return

        if path == "/favicon.ico":
            self.send_response(HTTPStatus.NO_CONTENT)
            self.end_headers()
            return

        self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found", "path": path})

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path

        try:
            body = self._read_json_body()

            if path == "/api/v1/settings/usb":
                response = self.server.controller.submit("apply_usb_settings", body)  # type: ignore[attr-defined]
                self._send_json(HTTPStatus.OK, response)
                return

            if path == "/api/v1/settings/sensor":
                response = self.server.controller.submit("apply_sensor_settings", body)  # type: ignore[attr-defined]
                self._send_json(HTTPStatus.OK, response)
                return

            if path == "/api/v1/actions":
                response = self.server.controller.submit("sensor_action", body)  # type: ignore[attr-defined]
                self._send_json(HTTPStatus.OK, response)
                return
        except Exception as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc), "snapshot": self.server.state.snapshot()})  # type: ignore[attr-defined]
            return

        self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found", "path": path})

    def _read_json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
        except Exception as exc:
            raise RuntimeError(f"Invalid JSON payload: {exc}") from exc
        if not isinstance(payload, dict):
            raise RuntimeError("JSON payload must be an object")
        return payload

    def _send_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
        self._send_bytes(status, "application/json; charset=utf-8", body)

    def _send_bytes(self, status: HTTPStatus, content_type: str, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt: str, *args: Any) -> None:
        logging.info("%s - %s", self.address_string(), fmt % args)


class WitHttpServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, host: str, port: int, state: StateStore, controller: SerialController) -> None:
        super().__init__((host, port), WitRequestHandler)
        self.state = state
        self.controller = controller


def build_kv(snapshot: dict[str, Any]) -> str:
    latest = snapshot.get("latest", {})
    mapping = {
        "connected": "1" if snapshot.get("connected") else "0",
        "stale": "1" if snapshot.get("stale") else "0",
        "timestamp_ms": str(snapshot.get("timestamp_ms", 0)),
        "age_ms": str(snapshot.get("age_ms", 0)),
        "sequence": str(snapshot.get("frames_received", 0)),
        "sample_rate_hz": _coerce_nan(snapshot.get("sample_rate_hz")),
        "roll_deg": _coerce_nan(latest.get("roll_deg")),
        "pitch_deg": _coerce_nan(latest.get("pitch_deg")),
        "yaw_deg": _coerce_nan(latest.get("yaw_deg")),
        "accel_x_g": _coerce_nan(latest.get("accel_x_g")),
        "accel_y_g": _coerce_nan(latest.get("accel_y_g")),
        "accel_z_g": _coerce_nan(latest.get("accel_z_g")),
        "gyro_x_dps": _coerce_nan(latest.get("gyro_x_dps")),
        "gyro_y_dps": _coerce_nan(latest.get("gyro_y_dps")),
        "gyro_z_dps": _coerce_nan(latest.get("gyro_z_dps")),
        "mag_x": _coerce_nan(latest.get("mag_x")),
        "mag_y": _coerce_nan(latest.get("mag_y")),
        "mag_z": _coerce_nan(latest.get("mag_z")),
        "temperature_c": _coerce_nan(latest.get("temperature_c")),
        "version_raw": _coerce_nan(latest.get("version_raw")),
        "device": snapshot.get("device_path", ""),
        "baud": str(snapshot.get("baud", 0)),
        "last_error": snapshot.get("last_error", ""),
    }
    return "\n".join(f"{key}={mapping[key]}" for key in KV_FIELDS) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="WitMotion IMU bridge for SystemCore")
    parser.add_argument("--device", default=_env_str("SC_WITMOTION_DEVICE", ""))
    parser.add_argument("--baud", type=int, default=_env_int("SC_WITMOTION_BAUD", 115200))
    parser.add_argument("--host", default=_env_str("SC_WITMOTION_HTTP_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=_env_int("SC_WITMOTION_HTTP_PORT", 9010))
    parser.add_argument("--stale-ms", type=int, default=_env_int("SC_WITMOTION_STALE_MS", DEFAULT_STALE_MS))
    parser.add_argument("--reconnect-ms", type=int, default=_env_int("SC_WITMOTION_RECONNECT_MS", 2000))
    parser.add_argument("--state-file", default=_env_str("SC_WITMOTION_STATE_FILE", STATE_FILE_DEFAULT))
    parser.add_argument("--log-level", default=_env_str("SC_WITMOTION_LOG_LEVEL", "INFO"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(message)s",
    )

    defaults = ControllerSettings(
        selected_device_path=_canonical_device_path(args.device),
        host_baud=args.baud,
        reconnect_ms=max(args.reconnect_ms, 250),
    )
    config_store = RuntimeConfigStore(args.state_file, defaults)
    settings = config_store.load()

    stop_event = threading.Event()
    state = StateStore(base_stale_ms=args.stale_ms)
    controller = SerialController(
        state=state,
        config_store=config_store,
        settings=settings,
        stop_event=stop_event,
    )
    controller.start()

    server = WitHttpServer(args.host, args.port, state, controller)

    def _shutdown(_signum: int, _frame: Any) -> None:
        logging.info("Shutdown signal received")
        stop_event.set()
        server.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    logging.info("HTTP server listening on %s:%d", args.host, args.port)
    try:
        server.serve_forever(poll_interval=0.5)
    finally:
        stop_event.set()
        server.server_close()
        controller.join(timeout=3.0)
        logging.info("Service stopped")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
