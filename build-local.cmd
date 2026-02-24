@echo off
setlocal EnableDelayedExpansion

REM Local Windows build for MicroPython firmware with pz_actuator C module
REM Requires: ESP-IDF v5.5.1 environment active, GNU Make in PATH

set "BOARD=ESP32_GENERIC_S3"
set "WORKSPACE=%~dp0."
set "IDF_DIR=C:\Espressif\frameworks\esp-idf-v5.5.1"

if not defined MICROPYTHON_DIR set "MICROPYTHON_DIR=%WORKSPACE%\..\micropython"

if not exist "%MICROPYTHON_DIR%\ports\esp32\Makefile" (
    echo Error: MicroPython not found at %MICROPYTHON_DIR%
    echo Set MICROPYTHON_DIR to your MicroPython clone
    exit /b 1
)

if not defined IDF_PATH (
    echo Activating ESP-IDF environment...
    call "%IDF_DIR%\export.bat"
    if errorlevel 1 (
        echo Error: Failed to activate ESP-IDF.
        exit /b 1
    )
)

REM mpy-cross on Windows has .exe extension; tell the build system explicitly
set "MICROPY_MPYCROSS=%MICROPYTHON_DIR%\mpy-cross\build\mpy-cross.exe"

where make >nul 2>&1
if errorlevel 1 (
    echo Error: GNU Make not found. Install via: choco install make
    exit /b 1
)

REM Convert backslashes to forward slashes for GNU Make (backslashes are
REM interpreted as escape characters by make and get stripped).
set "WS_FWD=%WORKSPACE:\=/%"

echo Building MicroPython firmware for %BOARD%...

pushd "%MICROPYTHON_DIR%\ports\esp32"
make submodules BOARD=%BOARD%
if errorlevel 1 ( popd & exit /b 1 )

make BOARD=%BOARD% USER_C_MODULES=%WS_FWD%/modules/micropython.cmake FROZEN_MANIFEST=%WS_FWD%/python/manifest.py
if errorlevel 1 ( popd & exit /b 1 )
popd

set "BUILD_DIR=%MICROPYTHON_DIR%\ports\esp32\build-%BOARD%"
set "OUT_DIR=%WORKSPACE%\build"
if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"

copy /Y "%BUILD_DIR%\micropython.bin" "%OUT_DIR%\" >nul
copy /Y "%BUILD_DIR%\bootloader\bootloader.bin" "%OUT_DIR%\" >nul
copy /Y "%BUILD_DIR%\partition_table\partition-table.bin" "%OUT_DIR%\" >nul

echo.
echo Build complete. Firmware copied to %OUT_DIR%\
echo   bootloader.bin     @ 0x0
echo   partition-table.bin @ 0x8000
echo   micropython.bin    @ 0x10000
