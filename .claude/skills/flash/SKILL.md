---
name: flash
description: Build firmware and flash to ESP32-S3. Handles bootloader entry, COM port detection, and power cycling.
user-invocable: true
---

# Flash Skill

Build optacon firmware and flash it to the ESP32-S3 board. Handles the full cycle: build → bootloader entry → flash → verify boot.

## Step 1: Determine if build is needed

Check what files changed since the last successful build:

```bash
git diff --name-only HEAD
git diff --name-only --cached
git ls-files --others --exclude-standard
```

- If ONLY non-frozen files changed (files in `web/`, `wifi_config.json`, config files — NOT in `python/` or `modules/`), **skip the build**. Those files go on the filesystem via mpremote, not in firmware.
- If ANY `modules/**/*.c`, `modules/**/*.h`, `python/**/*.py`, `python/manifest.py`, or build config (`micropython.cmake`, `CMakeLists.txt`, `Makefile`) changed → **full build needed**.
- If no changes at all but user explicitly asked to flash, build anyway (they may have built externally).

## Step 2: Build and enter bootloader IN PARALLEL

These two steps are independent — run them concurrently.

### 2a. Build (if needed — run in background)

```bash
./scripts/build.sh
```

ESP-IDF and MICROPYTHON_DIR are pre-configured in the shell environment. Run this in the background so you can enter bootloader while it builds.

### 2b. Enter bootloader (while build runs)

Enter bootloader directly — assume the board is in normal mode:
```
mcp__micropython__exec("import board_utils; board_utils.enter_bootloader()")
```
The I/O error response is expected — the board rebooted. No need to check board state first or poll for the bootloader port — `flash.sh` auto-detects it.

If the MCP call fails with a connection error (board not found), try power cycling via `mcp__siglent-spd__set_output` (CH1 OFF, wait 2s, CH1 ON, wait 3s), then retry.

## Step 3: Wait for build, then flash

1. Wait for the background build to complete. If it fails, report the error and stop.
2. Flash (no port argument needed — auto-detects bootloader via VID:PID filter):
   ```bash
   ./scripts/flash.sh
   ```
   The script uses `--port-filter vid=0x303a --port-filter pid=0x1001` and `--connect-attempts 10` to automatically find the bootloader port and retry connections. You can still pass an explicit port if needed: `./scripts/flash.sh /dev/ttyACM0`.
3. The serial exception after watchdog-reset at the end is **expected and normal** — ignore it. The flash succeeded if you see "Hash of data verified" before the exception.

## Step 4: Verify boot

1. Smoke test via MCP (auto-connects, retries internally):
   ```
   mcp__micropython__exec("import sys; print('MicroPython', sys.version)")
   ```
2. If smoke test passes, report success with the MicroPython version.
3. If it fails, wait a few seconds and retry once — the board may still be booting.
4. If still failing, try power cycling via `mcp__siglent-spd__set_output`.

## Error Recovery

- **Build fails**: Show the build error output. Common issues: disk full, C syntax errors, missing ESP-IDF environment.
- **Can't enter bootloader**: Board may be in a crash loop. Try power cycling via power supply MCP, or ask user for manual BOOT+RST.
- **Flash fails**: Board may have exited bootloader. Re-enter bootloader and retry.
- **Board doesn't boot after flash**: Power cycle. If still dead, the firmware may have a bug — suggest reverting to last known good build.
- **MCP exec hangs**: The serial port may be locked by another process.

## Notes

- The build takes ~5 minutes for a clean build, ~30 seconds for incremental.
- After flashing, the board resets via watchdog and reconnects as a new USB-CDC device.
- The port number often changes between normal mode and bootloader mode (e.g., ttyACM0 → ttyACM2). The auto-detect flags handle this.
