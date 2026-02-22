@echo off
REM Flash firmware to ESP32-S3
REM Usage: flash.cmd [COM_PORT]
REM Default: COM7

set PORT=%1
if "%PORT%"=="" set PORT=COM7

python -m esptool --chip esp32s3 -p %PORT% -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 4MB --flash_freq 80m 0x0 build/bootloader.bin 0x8000 build/partition-table.bin 0x10000 build/micropython.bin
