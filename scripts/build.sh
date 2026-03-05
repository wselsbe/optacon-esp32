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

# Apply OTA partition table config (idempotent)
OTA_SRC="${WORKSPACE}/config/sdkconfig.ota"
OTA_DST="boards/${BOARD}/sdkconfig.ota"
if [ -f "${OTA_SRC}" ]; then
    # Copy fragment to board directory
    cp "${OTA_SRC}" "${OTA_DST}"
    CMAKE_FILE="boards/${BOARD}/mpconfigboard.cmake"
    if ! grep -q "sdkconfig.ota" "${CMAKE_FILE}"; then
        sed -i "/sdkconfig\.board/a\\    boards/${BOARD}/sdkconfig.ota" "${CMAKE_FILE}"
    fi
    # Delete resolved sdkconfig if our fragment is newer, forcing Kconfig re-resolution
    BUILD_SDKCONFIG="build-${BOARD}/sdkconfig"
    if [ -f "${BUILD_SDKCONFIG}" ] && [ "${OTA_DST}" -nt "${BUILD_SDKCONFIG}" ]; then
        echo "sdkconfig.ota changed — forcing Kconfig re-resolution"
        rm -f "${BUILD_SDKCONFIG}"
    fi
fi

# Overlay our _boot.py onto vendor modules (removes stock filesystem boot.py
# on first format so frozen boot.py always runs for OTA rollback safety)
cp "${WORKSPACE}/python/_boot.py" "${MPY_DIR}/ports/esp32/modules/_boot.py"

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

# Copy OTA data initial image if present (OTA partition table only)
if [ -f "${BUILD_DIR}/ota_data_initial.bin" ]; then
    cp "${BUILD_DIR}/ota_data_initial.bin" "${OUT_DIR}/"
fi

echo ""
echo "Build complete. Firmware copied to ${OUT_DIR}/"
echo "  bootloader.bin      @ 0x0"
echo "  partition-table.bin @ 0x8000"
if [ -f "${OUT_DIR}/ota_data_initial.bin" ]; then
echo "  ota_data_initial.bin @ 0xd000"
fi
echo "  micropython.bin     @ 0x10000"
