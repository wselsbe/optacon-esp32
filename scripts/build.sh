#!/bin/bash
set -e

BOARD="${BOARD:-ESP32_GENERIC_S3}"
WORKSPACE="/workspace"
MPY_DIR="/opt/micropython"

# Build MicroPython with our C module and frozen Python
cd "${MPY_DIR}/ports/esp32"
make submodules BOARD="${BOARD}"
make BOARD="${BOARD}" \
    USER_C_MODULES="${WORKSPACE}/modules/micropython.cmake" \
    FROZEN_MANIFEST="${WORKSPACE}/python/manifest.py"

echo ""
echo "Build complete. Firmware at:"
echo "  ${MPY_DIR}/ports/esp32/build-${BOARD}/micropython.bin"
