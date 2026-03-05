# OTA Firmware Updates — Design

## Overview

Two-tier over-the-air update system for the Optacon ESP32-S3 firmware: full firmware binary updates via `esp32.OTA` and Python/web file updates via filesystem writes. Both tiers share a common update manifest and web UI.

## Architecture

### Two Update Tiers

1. **Firmware OTA** — writes complete firmware binary to standby OTA partition slot using MicroPython's `esp32.OTA` API. Auto-rollback via ESP-IDF bootloader if new firmware fails to boot.
2. **File OTA** — downloads individual Python/web files to filesystem with two-phase commit (download `.new` → verify → rename). Rollback via frozen `boot.py` watchdog.

### Push + Pull Delivery

- **Pull**: device fetches `versions.json` from a configurable HTTP base URL. Different URLs for different channels (stable, beta). No GitHub dependency — any static file server works.
- **Push**: manual file upload via web UI. Works on local network without internet.

### Frozen Boot Safety Net

A minimal `boot.py` is frozen into the firmware binary. It runs before any filesystem code and handles:
- Checking NVS `file_update` flag for failed file updates
- Restoring `.bak` files if a crash occurred after a file update
- Logging boot sequence and hardware init status
- Preserving previous boot log (`boot.log.prev`) for crash diagnostics

## Update Server

Static file hosting. No backend logic required.

### URL Structure

```
{base_url}/
├── versions.json
├── firmware/
│   └── {version}/micropython.bin
└── files/
    └── {version}/{path}
```

### versions.json

```json
{
  "latest_firmware": "1.1.0",
  "latest_files": "1.0.2",
  "firmware": {
    "1.1.0": {
      "url": "firmware/1.1.0/micropython.bin",
      "size": 1500000,
      "sha256": "abc123..."
    }
  },
  "files": {
    "1.0.2": {
      "changes": [
        {"path": "web_server.py", "url": "files/1.0.2/web_server.py", "sha256": "def456..."},
        {"path": "web/index.html", "url": "files/1.0.2/web/index.html", "sha256": "ghi789..."}
      ]
    }
  }
}
```

## Device Configuration

`ota_config.json` on filesystem:

```json
{
  "update_url": "https://updates.example.com/stable/",
  "diagnostics_url": "https://updates.example.com/diagnostics/",
  "firmware_version": "1.0.0",
  "files_version": "1.0.0",
  "auto_check": true
}
```

Channel is implied by the URL — switching channels means changing the URL.

## Firmware Update Flow

1. Fetch `versions.json`, compare `latest_firmware` vs local `firmware_version`
2. If newer: show in web UI, wait for user confirmation
3. Stream firmware binary → `esp32.OTA.write()` in 4 KB chunks
4. Verify SHA-256 of complete image
5. `esp32.OTA.end()` → sets boot to new slot
6. Update `ota_config.json` with new firmware version
7. `machine.reset()` → boots new firmware
8. New firmware calls `esp_ota_mark_app_valid_and_cancel_rollback()` after successful boot
9. If boot fails within timeout → bootloader automatically reverts to previous slot

## File Update Flow

1. Fetch `versions.json`, compare `latest_files` vs local `files_version`
2. If newer: show changed files in web UI, wait for confirmation
3. Download each changed file as `{path}.new`, verify SHA-256
4. If any verification fails → delete all `.new` files, abort
5. Set NVS `file_update=1` (watchdog flag)
6. Commit phase: rename `{path}` → `{path}.bak`, rename `{path}.new` → `{path}`
7. Update `ota_config.json` with new files version
8. `machine.soft_reset()`
9. Frozen `boot.py` checks NVS flag:
   - If `main.py` completes init successfully → clears `file_update=0`, deletes `.bak` files
   - If boot crashes → next reboot, `boot.py` restores all `.bak` files, reverts `files_version`

## Logging

Three log files with different lifecycles:

| File | Content | Lifecycle |
|------|---------|-----------|
| `boot.log` | Boot sequence, HW init (DRV2665, HV509, WiFi), OTA rollback actions | Overwritten each boot; previous saved as `boot.log.prev` |
| `ota_check.log` | Version check results with timestamps (from HTTP Date header) | Append, minimal entries only |
| `ota_update.log` | Download progress, SHA-256 verification, commit/rollback details | Append, rotate at 25 KB |

### boot.log example (normal boot)

```
[BOOT] Firmware: 1.1.0, Files: 1.0.2
[BOOT] OTA rollback check: no pending update
[BOOT] DRV2665: init OK, chip ID=0x02
[BOOT] HV509: SPI init OK, polarity GPIOs configured
[BOOT] WiFi: connected to "MyNetwork" (192.168.1.42)
[BOOT] Web server: started on port 80
[BOOT] OTA auto-check: update available (firmware 1.2.0)
```

### boot.log example (rollback boot)

```
[BOOT] Firmware: 1.1.0, Files: 1.0.2
[BOOT] OTA rollback check: file_update flag SET — rolling back
[BOOT] Rollback: web_server.py.bak → web_server.py
[BOOT] Rollback: wifi.py.bak → wifi.py
[BOOT] Rollback complete, files_version reverted to 1.0.1
[BOOT] DRV2665: init OK, chip ID=0x02
```

### ota_check.log example

```
[2026-03-05 10:30] Check OK — up to date (firmware=1.1.0, files=1.0.2)
[2026-03-05 14:15] Check OK — new firmware 1.2.0 detected
[2026-03-06 08:00] Check FAILED — connection timeout
```

### Diagnostics Upload

Manual-only "Send Diagnostics" button in web UI. POSTs all log files + device info to `diagnostics_url`. Button hidden if URL not configured.

## Web API

New endpoints added to existing Microdot web server:

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/ota/status` | GET | Current versions, last check time, update availability |
| `/api/ota/check` | POST | Trigger manual update check |
| `/api/ota/update/firmware` | POST | Start firmware OTA from update server |
| `/api/ota/update/files` | POST | Start file update from update server |
| `/api/ota/upload` | POST | Upload a .bin or .py file directly (push mode) |
| `/api/ota/log` | GET | Read log files (`?file=boot\|boot.prev\|check\|update`) |
| `/api/ota/config` | GET/PUT | Read/update OTA config |
| `/api/ota/diagnostics` | POST | Send logs to remote diagnostics endpoint |

### WebSocket Progress

Progress updates sent via existing `/ws` channel:

```json
{"type": "ota_progress", "phase": "download", "percent": 45, "bytes": 650000, "total": 1400000}
{"type": "ota_progress", "phase": "verify", "status": "ok"}
{"type": "ota_progress", "phase": "complete", "reboot_in": 3}
```

## Web UI

- **Updates page**: current version display, check for updates button, install buttons, download progress bar, log viewer, diagnostics send button
- **Version footer**: all web pages show firmware and files version in footer

## Update Check Timing

- Auto-check on boot after WiFi connects (if `auto_check` is true)
- Manual check via web UI "Check for Updates" button
- Notification shown in web UI if update is available (does not auto-install)

## Size Constraint

Current firmware is 1.8 MB; OTA partition slots are 1.5 MB each on 4 MB flash.

**Strategy**: optimize firmware size first (`-Os`, strip unused ESP-IDF components, reduce frozen modules). If 1.5 MB is not achievable, switch to ESP32-S3-WROOM-1 N8 module (8 MB flash, pin-compatible drop-in replacement).

## New/Modified Files

| File | Status | Description |
|------|--------|-------------|
| `python/boot.py` | New (frozen) | Boot safety net: NVS watchdog, rollback, HW init logging |
| `python/ota.py` | New (filesystem) | OTA update logic: check, download, verify, commit |
| `python/web_server.py` | Modified | Add `/api/ota/*` endpoints, WebSocket progress |
| `python/main.py` | Modified | Call OTA auto-check after boot, clear file_update flag |
| `python/manifest.py` | Modified | Add `boot.py` to frozen modules |
| `web/update.html` | New | Updates page with progress bar and log viewer |
| `web/index.html` | Modified | Add version footer, link to updates page |
| `web/wifi.html` | Modified | Add version footer |

## Decisions

- Channel is implied by update URL (no separate field)
- Server time from HTTP Date header used for timestamps (no RTC battery on device)
- Frozen `boot.py` is the safety net — survives filesystem corruption
- `boot.log.prev` preserved across reboots for crash diagnosis
- Diagnostics upload is manual-only (privacy)
- Auto-check notifies but never auto-installs
