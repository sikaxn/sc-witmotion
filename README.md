# sc-witmotion

`sc-witmotion` is a SystemCore package for a WitMotion IMU connected over USB serial.

It installs a systemd service that:

- reads WitMotion standard-protocol serial frames at `115200`
- hosts a browser UI and JSON/text API on port `9010`
- exposes a simple key/value endpoint for a WPILib Java client
- persists the selected USB port binding and sensor settings in `/var/lib/sc-witmotion/runtime.json`
- detects stale or disconnected serial links and automatically rescans/reconnects

The package defaults to the USB-to-serial adapter path observed on the target on July 16, 2026:

`/dev/serial/by-path/platform-xhci-hcd.1-usb-0:1:1.0-port0`

## Layout

- [package/build.sh](C:/Users/Nathan/Documents/GitHub/sc-witmotion/package/build.sh)
- [package/control/control](C:/Users/Nathan/Documents/GitHub/sc-witmotion/package/control/control)
- [package/overlay/etc/systemd/system/sc-witmotion.service](C:/Users/Nathan/Documents/GitHub/sc-witmotion/package/overlay/etc/systemd/system/sc-witmotion.service)
- [package/overlay/etc/default/sc-witmotion](C:/Users/Nathan/Documents/GitHub/sc-witmotion/package/overlay/etc/default/sc-witmotion)
- [package/overlay/usr/local/bin/sc-witmotion/sc_witmotion.py](C:/Users/Nathan/Documents/GitHub/sc-witmotion/package/overlay/usr/local/bin/sc-witmotion/sc_witmotion.py)
- [java-driver/src/main/java/com/systemcore/witmotion/ScWitMotionDriver.java](C:/Users/Nathan/Documents/GitHub/sc-witmotion/java-driver/src/main/java/com/systemcore/witmotion/ScWitMotionDriver.java)

## Build

From Windows with WSL Ubuntu:

```powershell
wsl.exe -d Ubuntu -- bash -lc "cd /mnt/c/Users/Nathan/Documents/GitHub/sc-witmotion/package && bash build.sh"
```

The output IPK is:

`package/sc-witmotion_0.1.0.ipk`

## Service API

The service listens on `http://systemcore.local:9010` or `http://172.30.0.1:9010` on the target network.

Endpoints:

- `/` browser dashboard
- `/healthz` health probe
- `/api/v1/status` JSON status and latest decoded values
- `/api/v1/imu.kv` plain-text key/value endpoint for the Java client
- `/api/v1/settings/usb` save the selected USB port binding and host baud
- `/api/v1/settings/sensor` apply sensor-side settings like output rate and content
- `/api/v1/actions` run actions like rescan and calibration commands

## WPILib Java Client

Copy [ScWitMotionDriver.java](C:/Users/Nathan/Documents/GitHub/sc-witmotion/java-driver/src/main/java/com/systemcore/witmotion/ScWitMotionDriver.java) into a robot project and start it from `robotInit()`.

Example:

```java
import com.systemcore.witmotion.ScWitMotionDriver;

public class Robot extends TimedRobot {
  private final ScWitMotionDriver imu = new ScWitMotionDriver();

  @Override
  public void robotInit() {
    imu.start();
  }

  @Override
  public void robotPeriodic() {
    var sample = imu.getLatestSample();
    if (sample.isFresh()) {
      System.out.println("yaw=" + sample.getYawDegrees());
    }
  }

  @Override
  public void close() {
    imu.close();
    super.close();
  }
}
```
