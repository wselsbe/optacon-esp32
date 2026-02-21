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

BUILD_DIR="${MPY_DIR}/ports/esp32/build-${BOARD}"
OUT_DIR="${WORKSPACE}/build"
mkdir -p "${OUT_DIR}"

cp "${BUILD_DIR}/micropython.bin" "${OUT_DIR}/"
cp "${BUILD_DIR}/bootloader/bootloader.bin" "${OUT_DIR}/"
cp "${BUILD_DIR}/partition_table/partition-table.bin" "${OUT_DIR}/"

echo ""
echo "Build complete. Firmware copied to /workspace/build/"
echo "  bootloader.bin    @ 0x0"
echo "  partition-table.bin @ 0x8000"
echo "  micropython.bin   @ 0x10000"
