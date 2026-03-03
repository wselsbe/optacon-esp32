@echo off
title Claude Code (optacon-pi)
:loop
ssh -t -R 9222:127.0.0.1:9222 optacon-pi "tmux attach -t claude"
echo.
echo Connection lost. Reconnecting in 10 seconds... (Ctrl+C to quit)
timeout /t 10 /nobreak >nul
goto loop
