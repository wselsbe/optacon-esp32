#!/bin/bash
set -e

PORT="${1:-/dev/ttyUSB0}"
BOARD="${BOARD:-ESP32_GENERIC_S3}"
MPY_DIR="/opt/micropython"

cd "${MPY_DIR}/ports/esp32"
make BOARD="${BOARD}" PORT="${PORT}" deploy
