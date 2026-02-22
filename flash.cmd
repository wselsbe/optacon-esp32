@echo off
REM Flash firmware to ESP32-S3
REM Usage: flash.cmd [COM_PORT]
REM Default: autodetect by VID

set PORT_ARG=--port-filter vid=0x303A
if not "%1"=="" set PORT_ARG=-p %1

python -m esptool --chip esp32s3 %PORT_ARG% -b 460800 --before default-reset --after watchdog-reset write-flash --flash-mode dio --flash-size 4MB --flash-freq 80m 0x0 build/bootloader.bin 0x8000 build/partition-table.bin 0x10000 build/micropython.bin
