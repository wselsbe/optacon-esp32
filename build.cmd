@echo off
REM Build MicroPython firmware with pz_actuator C module
set MSYS_NO_PATHCONV=1
docker compose run --rm dev bash /workspace/scripts/build.sh
