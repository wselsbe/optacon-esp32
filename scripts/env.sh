#!/bin/bash
# Source this file to set up the local build environment on Git Bash/MSYS2.
# Usage: source scripts/env.sh
#
# ESP-IDF's export.sh refuses to run on MSYS2, so we set paths manually.

export IDF_PATH="$HOME/esp/v5.5.1"

if [ ! -d "$IDF_PATH" ]; then
    echo "Error: ESP-IDF not found at $IDF_PATH"
    return 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export MICROPYTHON_DIR="${MICROPYTHON_DIR:-$(cd "$SCRIPT_DIR/../.." && pwd)/micropython}"

# ESP-IDF Python virtual environment
IDF_VENV="$HOME/.espressif/python_env/idf5.5_py3.13_env"
export VIRTUAL_ENV="$IDF_VENV"
export IDF_PYTHON_ENV_PATH="$IDF_VENV"

# ROM ELF files for debugging (optional, avoids cmake warning)
export ESP_ROM_ELF_DIR="$HOME/.espressif/tools/esp-rom-elfs/20241011"

# Build tool paths
IDF_TOOLS="$HOME/.espressif/tools"
export PATH="\
$IDF_VENV/Scripts:\
$IDF_TOOLS/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin:\
$IDF_TOOLS/cmake/3.30.2/bin:\
$IDF_TOOLS/ninja/1.12.1:\
$IDF_PATH/tools:\
$PATH"

echo "ESP-IDF environment ready (IDF_PATH=$IDF_PATH)"
echo "MicroPython: $MICROPYTHON_DIR"
