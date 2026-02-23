#!/bin/bash
set -e

BOARD="${BOARD:-ESP32_GENERIC_S3}"

# Auto-detect workspace (repo root) if not set
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE="${WORKSPACE:-$(cd "${SCRIPT_DIR}/.." && pwd)}"

# MicroPython directory: set MICROPYTHON_DIR in your environment,
# or fall back to /opt/micropython (Docker default)
MPY_DIR="${MICROPYTHON_DIR:-/opt/micropython}"

if [ ! -d "${MPY_DIR}/ports/esp32" ]; then
    echo "Error: MicroPython not found at ${MPY_DIR}"
    echo "Set MICROPYTHON_DIR to your MicroPython clone, e.g.:"
    echo "  export MICROPYTHON_DIR=C:/Projects/Optacon/micropython"
    exit 1
fi

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
echo "Build complete. Firmware copied to ${OUT_DIR}/"
echo "  bootloader.bin    @ 0x0"
echo "  partition-table.bin @ 0x8000"
echo "  micropython.bin   @ 0x10000"
