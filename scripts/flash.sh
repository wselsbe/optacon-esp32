#!/bin/bash
set -e

# Flash firmware to ESP32-S3
# Usage: ./scripts/flash.sh [PORT]
#   With port:    ./scripts/flash.sh COM3  or  /dev/ttyACM0
#   Without port: auto-detects ESP32-S3 in bootloader mode (VID:PID 303a:1001)

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/../build"

if [ ! -f "${BUILD_DIR}/micropython.bin" ]; then
    echo "Error: ${BUILD_DIR}/micropython.bin not found."
    echo "Run the build first: ./scripts/build.sh"
    exit 1
fi

PORT_ARGS=()
if [ -n "$1" ]; then
    PORT_ARGS=(-p "$1")
else
    # Auto-detect ESP32-S3 in bootloader mode
    PORT_ARGS=(--port-filter vid=0x303a --port-filter pid=0x1001)
    echo "Auto-detecting ESP32-S3 bootloader port..."
fi

# Include OTA data initial image if present
OTA_ARGS=()
if [ -f "${BUILD_DIR}/ota_data_initial.bin" ]; then
    OTA_ARGS=(0xd000 "${BUILD_DIR}/ota_data_initial.bin")
fi

python3 -m esptool --chip esp32s3 \
    "${PORT_ARGS[@]}" \
    -b 460800 \
    --before default_reset \
    --after watchdog_reset \
    --connect-attempts 10 \
    write_flash \
    --flash_mode dio \
    --flash_size 4MB \
    --flash_freq 80m \
    0x0     "${BUILD_DIR}/bootloader.bin" \
    0x8000  "${BUILD_DIR}/partition-table.bin" \
    "${OTA_ARGS[@]}" \
    0x10000 "${BUILD_DIR}/micropython.bin"
