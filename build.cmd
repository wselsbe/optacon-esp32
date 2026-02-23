@echo off
setlocal EnableDelayedExpansion

REM Build MicroPython firmware with pz_actuator C module
REM Usage: build.cmd [docker]
REM   build.cmd         - build locally (requires ESP-IDF + Make + MinGW)
REM   build.cmd docker  - build via Docker (no local toolchain needed)

if /i "%1"=="docker" (
    set MSYS_NO_PATHCONV=1
    docker compose run --rm dev bash /workspace/scripts/build.sh
    goto :eof
)

REM --- Local build ---
set "BOARD=ESP32_GENERIC_S3"
set "WORKSPACE=%~dp0."

REM MicroPython directory
if not defined MICROPYTHON_DIR set "MICROPYTHON_DIR=%WORKSPACE%\..\micropython"

if not exist "%MICROPYTHON_DIR%\ports\esp32\Makefile" (
    echo Error: MicroPython not found at %MICROPYTHON_DIR%
    echo Set MICROPYTHON_DIR to your MicroPython clone, e.g.:
    echo   set MICROPYTHON_DIR=C:\Projects\Optacon\micropython
    exit /b 1
)

REM Source ESP-IDF environment if not already set
if not defined IDF_PATH (
    if exist "%USERPROFILE%\esp\v5.5.1\export.bat" (
        echo Activating ESP-IDF environment...
        call "%USERPROFILE%\esp\v5.5.1\export.bat"
        if errorlevel 1 (
            echo Error: Failed to activate ESP-IDF. Run install.bat first.
            exit /b 1
        )
    ) else (
        echo Error: ESP-IDF not found at %USERPROFILE%\esp\v5.5.1
        exit /b 1
    )
)

REM Verify make is available
where make >nul 2>&1
if errorlevel 1 (
    echo Error: GNU Make not found. Install via: choco install make
    exit /b 1
)

echo Building MicroPython firmware for %BOARD%...

REM Build MicroPython with our C module and frozen Python
pushd "%MICROPYTHON_DIR%\ports\esp32"
make submodules BOARD=%BOARD%
if errorlevel 1 ( popd & exit /b 1 )

make BOARD=%BOARD% USER_C_MODULES="%WORKSPACE%\modules\micropython.cmake" FROZEN_MANIFEST="%WORKSPACE%\python\manifest.py"
if errorlevel 1 ( popd & exit /b 1 )
popd

REM Copy output binaries
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
