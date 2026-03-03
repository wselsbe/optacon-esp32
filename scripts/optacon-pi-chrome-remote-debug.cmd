@echo off
title Chrome Remote Debug
start "Chrome Remote Debug" "C:\Program Files\Google\Chrome\Application\chrome.exe" ^
  --remote-debugging-port=9222 ^
  --user-data-dir="%TEMP%\chrome-debug" ^
  --no-first-run ^
  --no-default-browser-check ^
  --window-size=1280,900
