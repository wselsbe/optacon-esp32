@echo off
REM Wrapper to run build-local.cmd with ESP-IDF environment.
REM Can be called from Git Bash: cmd.exe /C run-build.bat

call "C:\Espressif\frameworks\esp-idf-v5.5.1\export.bat" && call "C:\Projects\Optacon\optacon-firmware\build-local.cmd"
