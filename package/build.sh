#!/bin/bash

set -e

if [ ! -f "control/control" ]; then
    echo "Error: control/control not found"
    exit 1
fi

PACKAGE_NAME=$(grep "^Package:" control/control | cut -d' ' -f2- | tr -d ' ')
PACKAGE_VERSION=$(grep "^Version:" control/control | cut -d' ' -f2- | tr -d ' ')

if [ -z "$PACKAGE_NAME" ] || [ -z "$PACKAGE_VERSION" ]; then
    echo "Error: Package and Version must be set in control/control"
    exit 1
fi

PACKAGE_DIR="${PACKAGE_NAME}_${PACKAGE_VERSION}"
BUILD_DIR="build"
OUTPUT_FILE="${PACKAGE_NAME}_${PACKAGE_VERSION}.ipk"

echo "Building ${OUTPUT_FILE}"

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR/$PACKAGE_DIR"

cp -r overlay/* "$BUILD_DIR/$PACKAGE_DIR/"
mkdir -p "$BUILD_DIR/$PACKAGE_DIR/CONTROL"
cp control/* "$BUILD_DIR/$PACKAGE_DIR/CONTROL/"

find "$BUILD_DIR/$PACKAGE_DIR" -type d -name "__pycache__" -prune -exec rm -rf {} + 2>/dev/null || true
find "$BUILD_DIR/$PACKAGE_DIR" -name "*.pyc" -delete

find "$BUILD_DIR/$PACKAGE_DIR" -name "*.py" -exec chmod +x {} \;
find "$BUILD_DIR/$PACKAGE_DIR" -name "*.sh" -exec chmod +x {} \;
chmod +x "$BUILD_DIR/$PACKAGE_DIR/CONTROL"/* 2>/dev/null || true

cd "$BUILD_DIR"
tar --exclude='CONTROL' -czf data.tar.gz -C "$PACKAGE_DIR" .
tar -czf control.tar.gz -C "$PACKAGE_DIR/CONTROL" .
ar r "../${OUTPUT_FILE}" control.tar.gz data.tar.gz >/dev/null
cd ..

rm -rf "$BUILD_DIR"

echo "Built ${OUTPUT_FILE}"
