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
- If ANY `modules/**/*.c`, `modules/**/*.h`, `python/**/*.py`, `python/manifest.py`, or build config (`micropython.cmake`, `CMakeLists.txt`, `Makefile`) changed → **full Docker build needed**.
- If no changes at all but user explicitly asked to flash, build anyway (they may have built externally).

## Step 2: Build and enter bootloader IN PARALLEL

These two steps are independent — run them concurrently to save ~5 minutes.

### 2a. Build (if needed — run in background)

Use Docker exec for fast incremental builds:

```bash
docker compose up -d dev
MSYS_NO_PATHCONV=1 docker compose exec dev bash -c "source /opt/esp/idf/export.sh > /dev/null 2>&1 && bash /workspace/scripts/build.sh"
```

Run this in the background so you can enter bootloader while it builds.

### 2b. Enter bootloader (while build runs)

1. **Scan COM ports**:
   ```bash
   python -m serial.tools.list_ports -v
   ```

2. **Determine board state**:
   - **Normal mode** (VID:PID `303A:4001`): Enter bootloader:
     ```bash
     mpremote connect <PORT> exec "import board_utils; board_utils.enter_bootloader()"
     ```
   - **Already in bootloader** (VID:PID `303A:1001`): Skip — already ready for flashing.
   - **No board found**: Try power cycling via `mcp__siglent-spd__set_output` (channel 1, off then on, wait 3 seconds), then re-scan.
   - **Still not found**: Ask user to manually press BOOT+RST buttons or check USB cable.

3. **Verify bootloader COM port appeared** (VID:PID `303A:1001`):
   - The port number often changes when entering bootloader (e.g., COM5 → COM4).
   - Poll `python -m serial.tools.list_ports -v` up to 10 seconds.

## Step 3: Wait for build, then flash

1. Wait for the background build to complete. If it fails, report the error and stop.
2. Detect the bootloader COM port (VID:PID `303A:1001`).
3. Flash:
   ```bash
   ./scripts/flash.sh <BOOTLOADER_PORT>
   ```
4. The serial exception after watchdog-reset at the end is **expected and normal** — ignore it. The flash succeeded if you see "Hash of data verified" before the exception.

## Step 4: Verify boot

1. Wait for the normal COM port (VID:PID `303A:4001`) to reappear. Poll up to 15 seconds.
2. Quick smoke test:
   ```bash
   mpremote connect <PORT> exec "import sys; print('MicroPython', sys.version)"
   ```
3. If smoke test passes, report success with the MicroPython version.
4. If the board doesn't appear within 15 seconds, suggest power cycling.

## Error Recovery

- **Build fails**: Show the build error output. Common issues: Docker not running, disk full, C syntax errors.
- **Can't enter bootloader**: Board may be in a crash loop. Try power cycling via power supply MCP, or ask user for manual BOOT+RST.
- **Flash fails**: Wrong COM port, or board exited bootloader. Re-enter bootloader and retry.
- **Board doesn't boot after flash**: Power cycle. If still dead, the firmware may have a bug — suggest reverting to last known good build.
- **mpremote hangs**: Kill it after 10 seconds. The serial port may be locked by another process.

## Notes

- The build takes ~5 minutes for a clean build, ~30 seconds for incremental.
- Always use `docker compose exec` (not `run`) when the container is already running — much faster.
- The `MSYS_NO_PATHCONV=1` prefix is required in Git Bash to prevent path mangling.
- After flashing, the board resets via watchdog and reconnects as a new USB-CDC device.
