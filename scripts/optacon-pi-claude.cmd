@echo off
title Claude Code (optacon-pi)
ssh -t -R 9222:127.0.0.1:9222 optacon-pi "tmux attach -t claude"
