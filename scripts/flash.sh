#!/bin/bash
set -e

# Flash firmware from Windows host (or Linux/Mac)
# Usage: ./scripts/flash.sh [COM_PORT]
#   Windows: ./scripts/flash.sh COM3
#   Linux:   ./scripts/flash.sh /dev/ttyUSB0
#   Mac:     ./scripts/flash.sh /dev/cu.usbserial-0001
#
# Requires: pip install esptool

PORT="${1:?Usage: flash.sh <PORT> (e.g. COM3, /dev/ttyUSB0)}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/../build"

if [ ! -f "${BUILD_DIR}/micropython.bin" ]; then
    echo "Error: ${BUILD_DIR}/micropython.bin not found."
    echo "Run the build first: docker compose run --rm dev bash /workspace/scripts/build.sh"
    exit 1
fi

python -m esptool --chip esp32s3 \
    -p "${PORT}" \
    -b 460800 \
    --before default-reset \
    --after watchdog-reset \
    write-flash \
    --flash-mode dio \
    --flash-size 4MB \
    --flash-freq 80m \
    0x0     "${BUILD_DIR}/bootloader.bin" \
    0x8000  "${BUILD_DIR}/partition-table.bin" \
    0x10000 "${BUILD_DIR}/micropython.bin"
