# Optacon Firmware

MicroPython firmware for ESP32-S3 driving piezo actuators via DRV2665 + HV509 shift registers.

## Quick Start

```bash
docker compose up -d
docker compose exec dev bash
cd /workspace
./scripts/build.sh
./scripts/flash.sh /dev/ttyUSB0
```

## Architecture

- `modules/pz_actuator/` — C user module (DRV2665 I2C, shift register SPI, FreeRTOS background task)
- `python/` — MicroPython scripts (frozen into firmware)
- `scripts/` — Build and flash helpers
