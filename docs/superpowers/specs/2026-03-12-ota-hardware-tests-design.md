# OTA Hardware Tests Design

## Goal

Add hardware tests that verify OTA update functionality on real hardware: firmware updates, file updates, rollback on failure, and log event sequences. Tests run against the physical board with a mock OTA server on the test host.

## Decisions

- **Mock server:** Reuse `OTAMockServer` from `test/e2e/ota_server.py`, bound to `0.0.0.0` so the board can reach it over the network.
- **Server discovery:** Use `socket.gethostname()` / `socket.getfqdn()` to get the Pi's hostname, then resolve it to an IP. Simple and sufficient since the Pi and board are on the same LAN.
- **Firmware binary:** Use the latest locally built firmware (`micropython.bin`). Since `boot_cfg.FIRMWARE_VERSION` is frozen into the build, the version string won't change after update — tests verify the OTA mechanism (download, SHA verify, partition write, reboot), not the version bump. This is a known limitation documented in the tests.
- **Bad firmware:** Garbage binary (random bytes) with a matching SHA-256 in the manifest. This means `ota.py` downloads and flashes it successfully, but the ESP32 bootloader rejects the invalid image on boot and reverts to the previous partition. This tests the bootloader-level rollback path, not the download verification path.
- **File rollback trigger:** Serve a broken `web_server.py` (syntax error). The board's frozen `main.py` imports `web_server` at the top level (line 5), before `clear_update_flag()` (line 98). The import crash leaves the NVS flag set. A second reboot via `mpremote reset` (soft reset via serial) triggers `boot.py` rollback. Fallback: if soft reset fails, power cycle via PSU fixture.
- **Reboot detection:** Poll `GET /api/device/status` with 1s interval, up to 60s (90s for firmware rollback).
- **Test structure:** Single file (`test/hardware/test_ota.py`), sequential tests with shared session-scoped mock server. Tests are ordered by defining them in the desired execution order in the source file (pytest runs in file order by default).
- **Test isolation:** An autouse fixture calls `mock.reset()` before each test to prevent error mode / state leakage between tests. Each test sets its own error mode as needed.
- **Baseline capture:** A session-scoped fixture captures the board's initial state (firmware version, files version, OTA config) at session start. Rollback assertions compare against this baseline.
- **Teardown:** Session teardown restores the board's original `update_url` via `PUT /api/ota/config`.
- **No manual rollback API** — deferred to future work (see TODOs).
- **Log assertions:** Assert ordered event pattern sequences (not exact strings) in on-device log files via `GET /api/ota/log/{name}`.

## TODOs

- [ ] Add `/api/ota/rollback` endpoint for manual rollback to previous firmware partition (separate feature/spec)
- [ ] Add watchdog in `boot.py` that auto-reboots if `main.py` doesn't signal success within N seconds (would eliminate the need for `mpremote reset` in test 6)

## Test Infrastructure

### New files

- `test/hardware/test_ota.py` — all OTA hardware tests
- `test/hardware/fixtures/ota/bad_firmware.bin` — pre-generated garbage binary (~4 KB random bytes)
- `test/hardware/fixtures/ota/broken_web_server.py` — Python file with syntax error (e.g., `def !!!`)

### New fixtures (in `test/hardware/conftest.py`)

**`ota_server`** (session-scoped):
- Starts `OTAMockServer` on `0.0.0.0:<random-port>`
- Gets Pi's IP via `socket.gethostname()` resolution
- Yields `(mock, url)` where `url = "http://<pi-ip>:<port>"`
- Tears down after session

**`board_http`** (session-scoped):
- Wrapper around `urllib.request` for calling board HTTP API
- Methods with HTTP verb and path:
  - `get_device_status()` → `GET /api/device/status`
  - `get_ota_config()` → `GET /api/ota/config`
  - `put_ota_config(cfg)` → `PUT /api/ota/config` (JSON body)
  - `ota_check()` → `POST /api/ota/check`
  - `ota_update_firmware(manifest, version)` → `POST /api/ota/update/firmware` (JSON body: `{manifest, version}`)
  - `ota_update_files(manifest, version)` → `POST /api/ota/update/files` (JSON body: `{manifest, version}`)
  - `get_ota_log(name)` → `GET /api/ota/log/{name}` (name: `boot`, `boot.prev`, `check`, `update`)

**`wait_for_board(url, timeout)`**:
- Polls `GET /api/device/status` every 1s until HTTP 200 or timeout
- Returns True on success, raises on timeout

**`ota_baseline`** (session-scoped):
- Captures board's initial firmware version, files version, and OTA config
- Used by rollback assertions to verify restoration to original state

**`_reset_ota_server`** (autouse, function-scoped):
- Calls `mock.reset()` before each test to clear error mode and request log

### Fixture files

The mock server generates `versions.json` dynamically at test time with correct SHA-256 hashes. The firmware binary is copied from the latest local build output. Bad firmware and broken Python files are static fixtures checked into the repo.

## Test Scenarios

### Test execution order

Tests are defined in the source file in this order (pytest runs in file order):

1. `test_ota_check_detects_update` — verify check mechanism
2. `test_ota_firmware_update_happy_path` — firmware update + reboot
3. `test_ota_file_update_happy_path` — file update + soft reset
4. `test_ota_file_download_failure` — server error, no reboot
5. `test_ota_firmware_rollback_bad_binary` — bootloader rejects, auto-reverts
6. `test_ota_file_rollback_broken_file` — broken import, NVS rollback

Happy paths first (1-3), then failure without reboot (4), then rollback tests (5-6, most disruptive last).

### Test 1: `test_ota_check_detects_update`

1. `PUT /api/ota/config` — set `update_url` to mock server
2. `POST /api/ota/check`
3. Assert response: `firmware_available == True`
4. Check log: `GET /api/ota/log/check` — pattern sequence: `Check OK` → `detected`

### Test 2: `test_ota_firmware_update_happy_path`

1. `POST /api/ota/update/firmware` with manifest + version from check result
2. Poll board until responsive (60s timeout)
3. `GET /api/ota/log/update` — assert sequence: `Downloading firmware` → `SHA-256 verified` → `Set boot partition` → `Firmware update complete`
4. `GET /api/ota/log/boot` — assert normal boot (no rollback keywords)
5. Note: version string unchanged (same build) — documented limitation

### Test 3: `test_ota_file_update_happy_path`

1. `POST /api/ota/update/files` with manifest + version
2. Poll board until responsive (30s — soft reset)
3. `GET /api/ota/config` — assert `files_version == "0.2.0"`
4. `GET /api/ota/log/update` — assert sequence: `Downloading` → `All files downloaded` → `Installed` → `File update committed`
5. `GET /api/ota/log/boot` — assert no rollback keywords

### Test 4: `test_ota_file_download_failure`

1. Set `mock.error_mode = "500"`
2. `POST /api/ota/update/files`
3. Assert HTTP response indicates failure (no reboot triggered)
4. Board still responsive immediately
5. `GET /api/ota/log/update` — assert sequence: `Download FAILED` → `aborted`

### Test 5: `test_ota_firmware_rollback_bad_binary`

1. Override mock `versions_override` to point to `bad_firmware.bin` fixture with correct SHA-256 in manifest (so `ota.py` downloads and flashes successfully)
2. `POST /api/ota/update/firmware` — succeeds, board reboots
3. ESP32 bootloader rejects invalid image, reverts to previous partition
4. Poll board until responsive (90s — two boot attempts)
5. Assert board alive and functional
6. Note: ESP32 bootloader rollback is below MicroPython — won't appear in boot.log. Primary assertion is that the board recovers and is responsive.

### Test 6: `test_ota_file_rollback_broken_file`

1. Configure mock to serve broken `web_server.py` (syntax error) as a file update
2. `POST /api/ota/update/files` — succeeds, board soft-resets
3. Board crashes on `import web_server` — HTTP becomes unreachable
4. Send `mpremote reset` via serial to trigger second reboot (fallback: PSU power cycle)
5. Poll board until responsive (60s)
6. `GET /api/ota/config` — assert `files_version` reverted to baseline value
7. `GET /api/ota/log/boot` — assert sequence: `File update pending flag SET` → `Rollback:` → `Rollback complete`
