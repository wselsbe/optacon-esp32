# OTA Firmware Updates Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add two-tier OTA firmware updates (full firmware binary + Python file updates) with frozen boot.py safety net, auto-rollback, and web UI.

**Architecture:** Frozen `boot.py` handles rollback safety. Filesystem `ota.py` handles update logic. Web server gets `/api/ota/*` endpoints. Static `versions.json` manifest on any HTTP server enables pull updates; web UI upload enables push updates.

**Tech Stack:** MicroPython `esp32.Partition` API for firmware OTA, `esp32.NVS` for watchdog flag, `hashlib.sha256` for verification, Microdot async web framework for endpoints.

**Design doc:** `docs/plans/2026-03-05-ota-firmware-updates-design.md`

---

### Task 1: Enable OTA Partition Build Configuration

**Files:**
- Create: `config/sdkconfig.ota`
- Modify: `scripts/build.sh`

**Step 1: Create OTA sdkconfig override**

Create `config/sdkconfig.ota`:
```
# OTA partition table (two 1.5 MB app slots + otadata)
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions-4MiB-ota.csv"

# Size optimization to fit in 1.5 MB OTA slots
CONFIG_COMPILER_OPTIMIZATION_SIZE=y
CONFIG_COMPILER_OPTIMIZATION_PERF=n
CONFIG_COMPILER_OPTIMIZATION_CHECKS_SILENT=y
```

**Step 2: Modify build.sh to apply OTA config**

After line 22 (`cd "${MPY_DIR}/ports/esp32"`), add:

```bash
# Apply OTA partition table config (idempotent)
OTA_SRC="${WORKSPACE}/config/sdkconfig.ota"
OTA_DST="boards/${BOARD}/sdkconfig.ota"
if [ -f "${OTA_SRC}" ]; then
    cp "${OTA_SRC}" "${OTA_DST}"
    CMAKE_FILE="boards/${BOARD}/mpconfigboard.cmake"
    if ! grep -q "sdkconfig.ota" "${CMAKE_FILE}"; then
        sed -i "/sdkconfig\.board/a\\    boards/\${MICROPY_BOARD}/sdkconfig.ota" "${CMAKE_FILE}"
    fi
fi
```

**Step 3: Clean build to switch partition table**

A partition table change requires a full clean build:
```bash
cd ~/micropython/ports/esp32
make clean BOARD=ESP32_GENERIC_S3
```

Then build:
```bash
cd ~/projects/optacon-firmware && ./scripts/build.sh
```

**Step 4: Verify firmware size**

```bash
ls -la build/micropython.bin
```

Expected: under 1,572,864 bytes (1.5 MB = 0x180000). The `-Os` optimization should reduce from ~1.8 MB. If still too large, we need additional optimization (addressed in a follow-up task — see design doc for ESP32-S3 N8 module fallback).

**Step 5: Commit**

```bash
git add config/sdkconfig.ota scripts/build.sh
git commit -m "build: enable OTA partition table with size optimization"
```

---

### Task 2: Create Frozen boot.py (OTA Safety Net)

**Files:**
- Create: `python/boot.py`
- Modify: `python/manifest.py` (line 5)
- Modify: `python/main.py` (lines 1-8)

**Step 1: Create boot.py**

Create `python/boot.py`:

```python
"""Frozen boot safety net: OTA rollback, boot logging."""
import os

# Version constants (updated during OTA)
FIRMWARE_VERSION = "0.1.0"

_LOG = "/boot.log"
_LOG_PREV = "/boot.log.prev"


def _log(msg):
    """Print to REPL and append to boot.log."""
    print("[BOOT]", msg)
    try:
        with open(_LOG, "a") as f:
            f.write(msg + "\n")
    except Exception:
        pass


def _rotate_log():
    """Rename boot.log -> boot.log.prev, start fresh."""
    try:
        os.remove(_LOG_PREV)
    except OSError:
        pass
    try:
        os.rename(_LOG, _LOG_PREV)
    except OSError:
        pass


def _check_rollback():
    """Check NVS file_update flag. If set, restore .bak files."""
    import esp32

    nvs = esp32.NVS("ota")
    try:
        pending = nvs.get_i32("file_upd")
    except OSError:
        pending = 0

    if not pending:
        return

    _log("File update pending flag SET — rolling back")
    rolled = 0
    for name in os.listdir("/"):
        if name.endswith(".bak"):
            original = "/" + name[:-4]
            try:
                os.rename("/" + name, original)
                _log("Rollback: " + name + " -> " + name[:-4])
                rolled += 1
            except OSError as e:
                _log("Rollback FAILED: " + name + " " + str(e))

    # Try to revert files_version in ota_config.json
    try:
        import json

        with open("/ota_config.json") as f:
            cfg = json.load(f)
        if "prev_files_version" in cfg:
            cfg["files_version"] = cfg.pop("prev_files_version")
            with open("/ota_config.json", "w") as f:
                json.dump(cfg, f)
            _log("Reverted files_version to " + cfg["files_version"])
    except Exception:
        pass

    # Clear the flag
    nvs.set_i32("file_upd", 0)
    nvs.commit()
    _log("Rollback complete (" + str(rolled) + " files restored)")


def _read_ota_config():
    """Read OTA config, return dict or defaults."""
    try:
        import json

        with open("/ota_config.json") as f:
            return json.load(f)
    except Exception:
        return {}


# --- Boot sequence ---
_rotate_log()

cfg = _read_ota_config()
_log("Firmware: " + FIRMWARE_VERSION + ", Files: " + cfg.get("files_version", "unknown"))

_check_rollback()
```

**Step 2: Add boot.py to frozen manifest**

Modify `python/manifest.py` line 5:

```python
freeze(".", ("boot.py", "pz_drive_py.py", "drv2665.py", "shift_register.py", "main.py"))
```

**Step 3: Move DRV2665 standby init logging to main.py**

In `python/main.py`, after line 8 (`pz_drive.i2c_write(0x02, 0x40)`), add boot logging:

```python
pz_drive.i2c_write(0x02, 0x40)

# Log hardware init to boot.log
def _boot_log(msg):
    print("[BOOT]", msg)
    try:
        with open("/boot.log", "a") as f:
            f.write(msg + "\n")
    except Exception:
        pass

try:
    chip_id = pz_drive.i2c_read(0x02)
    _boot_log("DRV2665: init OK, reg2=0x{:02x}".format(chip_id))
except Exception as e:
    _boot_log("DRV2665: init FAILED " + str(e))
```

**Step 4: Verify boot.py runs on the board**

After building and flashing, check REPL output. Expected:
```
[BOOT] Firmware: 0.1.0, Files: unknown
[BOOT] DRV2665: init OK, reg2=0x40
```

Check `/boot.log` on filesystem:
```python
with open("/boot.log") as f:
    print(f.read())
```

**Step 5: Commit**

```bash
git add python/boot.py python/manifest.py python/main.py
git commit -m "feat: add frozen boot.py OTA safety net with rollback and logging"
```

---

### Task 3: Create ota.py (OTA Core Module)

This is the largest task — the core OTA logic. It lives on the filesystem so it can be updated without reflashing.

**Files:**
- Create: `python/ota.py`

**Step 1: Create ota.py with config management and logging**

```python
"""OTA update manager — check, download, verify, apply updates."""
import json
import os

import hashlib

_CHECK_LOG = "/ota_check.log"
_UPDATE_LOG = "/ota_update.log"
_CONFIG_FILE = "/ota_config.json"
_MAX_LOG_SIZE = 25 * 1024  # 25 KB


def _log_check(msg):
    """Append to OTA check log."""
    print("[OTA-CHECK]", msg)
    _append_log(_CHECK_LOG, msg)


def _log_update(msg):
    """Append to OTA update log."""
    print("[OTA-UPDATE]", msg)
    _append_log(_UPDATE_LOG, msg)


def _append_log(path, msg):
    """Append message to log file with rotation."""
    try:
        try:
            size = os.stat(path)[6]
        except OSError:
            size = 0
        if size > _MAX_LOG_SIZE:
            # Truncate: keep last half
            with open(path) as f:
                f.read(size // 2)  # skip first half
                keep = f.read()
            with open(path, "w") as f:
                f.write(keep)
        with open(path, "a") as f:
            f.write(msg + "\n")
    except Exception:
        pass


def load_config():
    """Load OTA config from filesystem."""
    try:
        with open(_CONFIG_FILE) as f:
            return json.load(f)
    except Exception:
        return {
            "update_url": "",
            "diagnostics_url": "",
            "firmware_version": "0.1.0",
            "files_version": "0.1.0",
            "auto_check": True,
        }


def save_config(cfg):
    """Save OTA config to filesystem."""
    with open(_CONFIG_FILE, "w") as f:
        json.dump(cfg, f)


def _http_get(url, headers=None):
    """Minimal HTTP GET returning (status, headers_dict, socket).

    Returns the raw socket for streaming. Caller must close it.
    """
    import socket

    try:
        import ssl
    except ImportError:
        ssl = None

    # Parse URL
    proto, _, host_path = url.split("/", 2)
    use_ssl = proto == "https:"
    host_port, path = host_path.split("/", 1) if "/" in host_path else (host_path, "")
    path = "/" + path
    if ":" in host_port:
        host, port = host_port.rsplit(":", 1)
        port = int(port)
    else:
        host = host_port
        port = 443 if use_ssl else 80

    # Connect
    addr = socket.getaddrinfo(host, port)[0][-1]
    s = socket.socket()
    s.settimeout(30)
    s.connect(addr)
    if use_ssl and ssl:
        s = ssl.wrap_socket(s, server_hostname=host)

    # Send request
    req = "GET {} HTTP/1.0\r\nHost: {}\r\n".format(path, host)
    if headers:
        for k, v in headers.items():
            req += "{}: {}\r\n".format(k, v)
    req += "\r\n"
    s.write(req.encode())

    # Read status line
    line = s.readline().decode()
    status = int(line.split()[1])

    # Read headers
    resp_headers = {}
    while True:
        line = s.readline().decode().strip()
        if not line:
            break
        if ":" in line:
            k, v = line.split(":", 1)
            resp_headers[k.strip().lower()] = v.strip()

    return status, resp_headers, s


def check_for_updates(notify_cb=None):
    """Check update server for new versions.

    Args:
        notify_cb: optional callback(fw_version, files_version) called if update available

    Returns dict: {firmware_available, files_available, firmware_version, files_version}
    """
    cfg = load_config()
    base_url = cfg.get("update_url", "")
    if not base_url:
        _log_check("No update URL configured")
        return None

    url = base_url.rstrip("/") + "/versions.json"
    try:
        status, headers, sock = _http_get(url)
        if status != 200:
            _log_check("Check FAILED — HTTP " + str(status))
            sock.close()
            return None

        data = b""
        while True:
            chunk = sock.read(4096)
            if not chunk:
                break
            data += chunk
        sock.close()

        # Extract server date from headers
        date_str = headers.get("date", "")
        manifest = json.loads(data)
    except Exception as e:
        _log_check("Check FAILED — " + str(e))
        return None

    latest_fw = manifest.get("latest_firmware", "")
    latest_files = manifest.get("latest_files", "")
    current_fw = cfg.get("firmware_version", "0.0.0")
    current_files = cfg.get("files_version", "0.0.0")

    fw_available = latest_fw and latest_fw != current_fw
    files_available = latest_files and latest_files != current_files

    # Log result
    prefix = "[" + date_str + "] " if date_str else ""
    if fw_available or files_available:
        parts = []
        if fw_available:
            parts.append("new firmware " + latest_fw)
        if files_available:
            parts.append("new files " + latest_files)
        _log_check(prefix + "Check OK — " + ", ".join(parts) + " detected")
    else:
        _log_check(
            prefix
            + "Check OK — up to date (firmware="
            + current_fw
            + ", files="
            + current_files
            + ")"
        )

    result = {
        "firmware_available": fw_available,
        "files_available": files_available,
        "latest_firmware": latest_fw,
        "latest_files": latest_files,
        "current_firmware": current_fw,
        "current_files": current_files,
        "manifest": manifest,
    }

    if notify_cb and (fw_available or files_available):
        notify_cb(latest_fw if fw_available else None, latest_files if files_available else None)

    return result


def update_firmware(manifest, version, progress_cb=None):
    """Download and flash firmware update.

    Args:
        manifest: parsed versions.json dict
        version: firmware version string to install
        progress_cb: optional callback(bytes_done, bytes_total)

    Returns True on success (caller should reboot).
    """
    import esp32
    import machine

    fw_info = manifest.get("firmware", {}).get(version)
    if not fw_info:
        _log_update("Firmware version " + version + " not found in manifest")
        return False

    cfg = load_config()
    base_url = cfg.get("update_url", "").rstrip("/")
    url = base_url + "/" + fw_info["url"]
    expected_size = fw_info.get("size", 0)
    expected_sha = fw_info.get("sha256", "")

    _log_update("Downloading firmware " + version + " from " + url)
    _log_update("Expected size: " + str(expected_size) + " bytes")

    try:
        status, headers, sock = _http_get(url)
        if status != 200:
            _log_update("Download FAILED — HTTP " + str(status))
            sock.close()
            return False

        # Get next OTA partition
        cur = esp32.Partition(esp32.Partition.RUNNING)
        nxt = cur.get_next_update()
        _log_update("Writing to partition: " + nxt.info()[4])

        # Stream to partition
        sha = hashlib.sha256()
        block = 0
        total = 0
        while True:
            chunk = sock.read(4096)
            if not chunk:
                break
            # Pad last block to 4096 if needed
            if len(chunk) < 4096:
                padded = chunk + b"\xff" * (4096 - len(chunk))
                nxt.writeblocks(block, padded)
            else:
                nxt.writeblocks(block, chunk)
            sha.update(chunk)
            block += 1
            total += len(chunk)
            if progress_cb:
                progress_cb(total, expected_size)

        sock.close()
        _log_update("Downloaded " + str(total) + " bytes")

        # Verify SHA-256
        actual_sha = sha.digest().hex()  # binascii.hexlify(sha.digest()).decode()
        if expected_sha and actual_sha != expected_sha:
            _log_update("SHA-256 MISMATCH! Expected: " + expected_sha + " Got: " + actual_sha)
            return False
        _log_update("SHA-256 verified OK")

        # Set as next boot partition
        nxt.set_boot()
        _log_update("Set boot partition to " + nxt.info()[4])

        # Update config
        cfg["firmware_version"] = version
        save_config(cfg)
        _log_update("Firmware update complete — reboot to activate")

        return True

    except Exception as e:
        _log_update("Firmware update FAILED — " + str(e))
        return False


def update_files(manifest, version, progress_cb=None):
    """Download and apply file updates with two-phase commit.

    Args:
        manifest: parsed versions.json dict
        version: files version string to install
        progress_cb: optional callback(file_index, total_files, filename)

    Returns True on success (caller should soft-reset).
    """
    import esp32

    files_info = manifest.get("files", {}).get(version)
    if not files_info:
        _log_update("Files version " + version + " not found in manifest")
        return False

    changes = files_info.get("changes", [])
    if not changes:
        _log_update("No file changes in version " + version)
        return False

    cfg = load_config()
    base_url = cfg.get("update_url", "").rstrip("/")

    _log_update("Updating " + str(len(changes)) + " files to version " + version)

    # Phase 1: Download all .new files and verify
    downloaded = []
    try:
        for i, change in enumerate(changes):
            path = "/" + change["path"]
            url = base_url + "/" + change["url"]
            expected_sha = change.get("sha256", "")
            new_path = path + ".new"

            _log_update("Downloading " + change["path"])
            if progress_cb:
                progress_cb(i, len(changes), change["path"])

            status, headers, sock = _http_get(url)
            if status != 200:
                _log_update("Download FAILED — HTTP " + str(status) + " for " + change["path"])
                sock.close()
                raise Exception("HTTP " + str(status))

            sha = hashlib.sha256()
            with open(new_path, "wb") as f:
                while True:
                    chunk = sock.read(4096)
                    if not chunk:
                        break
                    f.write(chunk)
                    sha.update(chunk)
            sock.close()

            actual_sha = sha.digest().hex()
            if expected_sha and actual_sha != expected_sha:
                _log_update("SHA-256 MISMATCH for " + change["path"])
                raise Exception("SHA-256 mismatch: " + change["path"])

            downloaded.append(path)

    except Exception as e:
        # Cleanup .new files
        _log_update("File update aborted — " + str(e))
        for path in downloaded:
            try:
                os.remove(path + ".new")
            except OSError:
                pass
        return False

    _log_update("All files downloaded and verified")

    # Phase 2: Set NVS watchdog flag
    nvs = esp32.NVS("ota")
    nvs.set_i32("file_upd", 1)
    nvs.commit()

    # Save previous version for rollback
    cfg["prev_files_version"] = cfg.get("files_version", "0.0.0")

    # Phase 3: Atomic rename (backup originals, install new)
    for path in downloaded:
        bak_path = path + ".bak"
        new_path = path + ".new"
        try:
            os.remove(bak_path)
        except OSError:
            pass
        try:
            os.rename(path, bak_path)
        except OSError:
            pass  # file may not exist yet
        os.rename(new_path, path)
        _log_update("Installed: " + path)

    # Phase 4: Update config
    cfg["files_version"] = version
    save_config(cfg)
    _log_update("File update committed — reboot to activate")

    return True


def clear_update_flag():
    """Called by main.py after successful boot to clear the NVS watchdog."""
    import esp32

    nvs = esp32.NVS("ota")
    try:
        pending = nvs.get_i32("file_upd")
    except OSError:
        pending = 0
    if pending:
        nvs.set_i32("file_upd", 0)
        nvs.commit()
        # Clean up .bak files
        for name in os.listdir("/"):
            if name.endswith(".bak"):
                try:
                    os.remove("/" + name)
                except OSError:
                    pass


def mark_firmware_valid():
    """Mark current firmware as valid (cancel rollback)."""
    import esp32

    try:
        cur = esp32.Partition(esp32.Partition.RUNNING)
        cur.mark_app_valid_cancel_rollback()
    except Exception:
        pass  # not in OTA mode or already marked


def upload_firmware(data, size):
    """Flash uploaded firmware binary.

    Args:
        data: bytes or file-like object with .read(n)
        size: total expected size

    Returns True on success.
    """
    import esp32

    try:
        cur = esp32.Partition(esp32.Partition.RUNNING)
        nxt = cur.get_next_update()
        _log_update("Upload firmware: writing to " + nxt.info()[4])

        sha = hashlib.sha256()
        block = 0
        total = 0
        if isinstance(data, bytes):
            # Write all at once in blocks
            for i in range(0, len(data), 4096):
                chunk = data[i : i + 4096]
                if len(chunk) < 4096:
                    chunk = chunk + b"\xff" * (4096 - len(chunk))
                nxt.writeblocks(block, chunk)
                sha.update(data[i : i + 4096])  # hash unpadded
                block += 1
                total += min(4096, len(data) - i)
        else:
            while True:
                chunk = data.read(4096)
                if not chunk:
                    break
                raw = chunk
                if len(chunk) < 4096:
                    chunk = chunk + b"\xff" * (4096 - len(chunk))
                nxt.writeblocks(block, chunk)
                sha.update(raw)
                block += 1
                total += len(raw)

        nxt.set_boot()
        _log_update("Upload firmware complete: " + str(total) + " bytes, SHA=" + sha.digest().hex())
        return True

    except Exception as e:
        _log_update("Upload firmware FAILED — " + str(e))
        return False


def upload_file(path, data):
    """Write an uploaded file to filesystem.

    Args:
        path: destination path (e.g. "/web_server.py")
        data: bytes content
    """
    _log_update("Upload file: " + path + " (" + str(len(data)) + " bytes)")
    with open(path, "wb") as f:
        f.write(data)


def get_status():
    """Return OTA status dict for the web API."""
    cfg = load_config()
    import boot

    return {
        "firmware_version": boot.FIRMWARE_VERSION,
        "files_version": cfg.get("files_version", "unknown"),
        "update_url": cfg.get("update_url", ""),
        "auto_check": cfg.get("auto_check", True),
        "diagnostics_url": cfg.get("diagnostics_url", ""),
    }


def get_log(name):
    """Read a log file by name."""
    paths = {
        "boot": "/boot.log",
        "boot.prev": "/boot.log.prev",
        "check": _CHECK_LOG,
        "update": _UPDATE_LOG,
    }
    path = paths.get(name, "")
    if not path:
        return ""
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return ""


def send_diagnostics():
    """POST all logs + device info to diagnostics URL."""
    import gc
    import time

    import boot

    cfg = load_config()
    url = cfg.get("diagnostics_url", "")
    if not url:
        return False

    diag = {
        "firmware_version": boot.FIRMWARE_VERSION,
        "files_version": cfg.get("files_version", "unknown"),
        "free_mem": gc.mem_free(),
        "boot_log": get_log("boot"),
        "boot_log_prev": get_log("boot.prev"),
        "check_log": get_log("check"),
        "update_log": get_log("update"),
    }

    try:
        import socket

        try:
            import ssl
        except ImportError:
            ssl = None

        # Parse URL
        proto, _, host_path = url.split("/", 2)
        use_ssl = proto == "https:"
        host_port, path = host_path.split("/", 1) if "/" in host_path else (host_path, "")
        path = "/" + path
        if ":" in host_port:
            host, port = host_port.rsplit(":", 1)
            port = int(port)
        else:
            host = host_port
            port = 443 if use_ssl else 80

        body = json.dumps(diag)
        addr = socket.getaddrinfo(host, port)[0][-1]
        s = socket.socket()
        s.settimeout(30)
        s.connect(addr)
        if use_ssl and ssl:
            s = ssl.wrap_socket(s, server_hostname=host)

        req = "POST {} HTTP/1.0\r\nHost: {}\r\nContent-Type: application/json\r\nContent-Length: {}\r\n\r\n".format(
            path, host, len(body)
        )
        s.write(req.encode())
        s.write(body.encode())

        line = s.readline().decode()
        status = int(line.split()[1])
        s.close()

        _log_update("Diagnostics sent — HTTP " + str(status))
        return status == 200

    except Exception as e:
        _log_update("Diagnostics send FAILED — " + str(e))
        return False
```

**Step 2: Verify ota.py loads on the board**

Upload `python/ota.py` to the board filesystem and test basic functions:

```python
import ota
cfg = ota.load_config()
print(cfg)
status = ota.get_status()
print(status)
```

**Step 3: Commit**

```bash
git add python/ota.py
git commit -m "feat: add OTA core module with check, download, verify, and rollback"
```

---

### Task 4: Add OTA Endpoints to Web Server

**Files:**
- Modify: `python/web_server.py` (add routes after line 128)

**Step 1: Add OTA imports and routes**

At the top of `web_server.py`, add import (after line 2):
```python
import ota
```

After the `/wifi/status` route (after line 128), add the OTA API routes:

```python
# --- OTA API ---

@app.route("/api/ota/status")
async def ota_status(request):
    return json.dumps(ota.get_status()), 200, {"Content-Type": "application/json"}


@app.route("/api/ota/config", methods=["GET", "PUT"])
async def ota_config(request):
    if request.method == "GET":
        return json.dumps(ota.load_config()), 200, {"Content-Type": "application/json"}
    # PUT: update config
    data = json.loads(request.body.decode())
    cfg = ota.load_config()
    for key in ("update_url", "diagnostics_url", "auto_check"):
        if key in data:
            cfg[key] = data[key]
    ota.save_config(cfg)
    return json.dumps(cfg), 200, {"Content-Type": "application/json"}


@app.route("/api/ota/check", methods=["POST"])
async def ota_check(request):
    result = ota.check_for_updates()
    if result is None:
        return json.dumps({"error": "Check failed"}), 500, {"Content-Type": "application/json"}
    return json.dumps(result), 200, {"Content-Type": "application/json"}


@app.route("/api/ota/update/firmware", methods=["POST"])
async def ota_update_firmware(request):
    data = json.loads(request.body.decode())
    version = data.get("version")
    manifest = data.get("manifest")
    if not version or not manifest:
        return json.dumps({"error": "version and manifest required"}), 400, {
            "Content-Type": "application/json"
        }

    def progress(done, total):
        # WebSocket progress is sent separately if a WS client is connected
        pass

    ok = ota.update_firmware(manifest, version, progress_cb=progress)
    if ok:
        return (
            json.dumps({"status": "ok", "message": "Firmware updated. Rebooting..."}),
            200,
            {"Content-Type": "application/json"},
        )
    return json.dumps({"error": "Firmware update failed"}), 500, {"Content-Type": "application/json"}


@app.route("/api/ota/update/files", methods=["POST"])
async def ota_update_files(request):
    data = json.loads(request.body.decode())
    version = data.get("version")
    manifest = data.get("manifest")
    if not version or not manifest:
        return json.dumps({"error": "version and manifest required"}), 400, {
            "Content-Type": "application/json"
        }
    ok = ota.update_files(manifest, version)
    if ok:
        return (
            json.dumps({"status": "ok", "message": "Files updated. Rebooting..."}),
            200,
            {"Content-Type": "application/json"},
        )
    return json.dumps({"error": "File update failed"}), 500, {"Content-Type": "application/json"}


@app.route("/api/ota/upload", methods=["POST"])
async def ota_upload(request):
    # Content-Type determines behavior:
    # application/octet-stream → firmware binary
    # multipart/form-data or filename param → file upload
    ct = request.headers.get("Content-Type", "")
    filename = request.args.get("filename", "")

    if filename:
        # File upload to filesystem
        path = "/" + filename.lstrip("/")
        ota.upload_file(path, request.body)
        return json.dumps({"status": "ok", "path": path}), 200, {"Content-Type": "application/json"}
    else:
        # Firmware binary upload
        ok = ota.upload_firmware(request.body, len(request.body))
        if ok:
            return (
                json.dumps({"status": "ok", "message": "Firmware uploaded. Rebooting..."}),
                200,
                {"Content-Type": "application/json"},
            )
        return json.dumps({"error": "Upload failed"}), 500, {"Content-Type": "application/json"}


@app.route("/api/ota/log")
async def ota_log(request):
    name = request.args.get("file", "boot")
    content = ota.get_log(name)
    return json.dumps({"log": content}), 200, {"Content-Type": "application/json"}


@app.route("/api/ota/diagnostics", methods=["POST"])
async def ota_diagnostics(request):
    ok = ota.send_diagnostics()
    if ok:
        return json.dumps({"status": "ok"}), 200, {"Content-Type": "application/json"}
    return json.dumps({"error": "Failed to send diagnostics"}), 500, {
        "Content-Type": "application/json"
    }


@app.route("/update")
async def update_page(request):
    return send_file("/web/update.html")
```

**Step 2: Verify endpoints work**

After uploading modified web_server.py, reboot and test:
```
curl http://esp-optacon.local/api/ota/status
curl http://esp-optacon.local/api/ota/log?file=boot
```

**Step 3: Commit**

```bash
git add python/web_server.py
git commit -m "feat: add OTA REST API endpoints to web server"
```

---

### Task 5: Create Updates Web Page

**Files:**
- Create: `web/update.html`

**Step 1: Create update.html**

Create `web/update.html` following the existing design patterns from `wifi.html` and `docs.html`. Must include:

- Header with OPTACON brand + "Control Panel" back link (same as wifi.html)
- **Status card**: firmware version, files version, update URL, last check time
- **Available updates card** (hidden when no updates): shows what's available with Install buttons
- **Manual upload card**: file input for .bin or .py upload with Upload button
- **Settings card**: update URL input, auto-check toggle, diagnostics URL, Save button
- **Log viewer card**: tabs for boot/boot.prev/check/update logs, auto-refresh
- **Progress overlay**: shown during download with progress bar and status text
- **Version footer**: firmware + files version

Key JavaScript behaviors:
- On page load: `GET /api/ota/status` to populate versions
- "Check for Updates" button: `POST /api/ota/check`
- "Install Firmware": `POST /api/ota/update/firmware` with manifest + version
- "Install Files": `POST /api/ota/update/files` with manifest + version
- "Upload": `POST /api/ota/upload` with file content
- "Send Diagnostics": `POST /api/ota/diagnostics`
- Log tabs: `GET /api/ota/log?file=<name>`
- Save settings: `PUT /api/ota/config`
- After firmware update success: show "Rebooting..." message, auto-reload after 10s
- After file update success: show "Rebooting..." message, auto-reload after 5s

Use the same CSS variables, card styles, button styles, and font stack as the existing pages. Match the visual pattern of wifi.html for form inputs and docs.html for the footer.

**Step 2: Verify page loads**

Upload to board and navigate to `http://esp-optacon.local/update`. Verify:
- Status card shows version info
- Check button works (returns "no URL configured" if no update_url set)
- Log viewer loads boot.log content
- Settings form saves to ota_config.json

**Step 3: Commit**

```bash
git add web/update.html
git commit -m "feat: add OTA updates web page"
```

---

### Task 6: Add Version Footer to All Web Pages

**Files:**
- Modify: `web/index.html` (before `</body>`, after line 490)
- Modify: `web/wifi.html` (before `</script></body>`, after line 132)
- Modify: `web/docs.html` (line 464-466, existing footer)
- Modify: `web/update.html` (already has footer from Task 5)

**Step 1: Add footer CSS and HTML to index.html**

Before line 490 (`</script>`), add a footer div and JS to populate it:

After the `<div class="toast">` (line 288), add:
```html
<div class="footer" id="footer" style="text-align:center;padding:16px 0 4px;font-size:11px;color:var(--text3)">
  <a href="/update" style="color:var(--esp);text-decoration:none">Updates</a>
  &middot; <span id="ver-info"></span>
</div>
```

In the `<script>` section, add after `connect()` (line 489):
```javascript
fetch('/api/ota/status').then(function(r){return r.json()}).then(function(d){
  var v='FW '+d.firmware_version;
  if(d.files_version)v+=' · Files '+d.files_version;
  document.getElementById('ver-info').textContent=v;
}).catch(function(){});
```

**Step 2: Add footer to wifi.html**

Before `</body>` (line 134), add:
```html
<div id="footer" style="text-align:center;padding:16px 0 4px;font-size:11px;color:var(--text3)">
  <a href="/update" style="color:var(--esp);text-decoration:none">Updates</a>
  &middot; <span id="ver-info"></span>
</div>
<script>
fetch('/api/ota/status').then(function(r){return r.json()}).then(function(d){
  var v='FW '+d.firmware_version;
  if(d.files_version)v+=' · Files '+d.files_version;
  document.getElementById('ver-info').textContent=v;
}).catch(function(){});
</script>
```

**Step 3: Add version + Updates link to docs.html footer**

Modify `web/docs.html` lines 464-466. Replace:
```html
<div class="footer">
  <a href="/">Control Panel</a> &middot; <a href="/wifi">WiFi Settings</a>
</div>
```
With:
```html
<div class="footer">
  <a href="/">Control Panel</a> &middot; <a href="/wifi">WiFi Settings</a> &middot; <a href="/update">Updates</a>
  <div id="ver-info" style="margin-top:4px"></div>
</div>
<script>
fetch('/api/ota/status').then(function(r){return r.json()}).then(function(d){
  var v='FW '+d.firmware_version;
  if(d.files_version)v+=' · Files '+d.files_version;
  document.getElementById('ver-info').textContent=v;
}).catch(function(){});
</script>
```

**Step 4: Verify footers appear on all pages**

Navigate to each page and confirm version footer is visible.

**Step 5: Commit**

```bash
git add web/index.html web/wifi.html web/docs.html
git commit -m "feat: add version footer and Updates link to all web pages"
```

---

### Task 7: Integrate OTA in Boot Sequence

**Files:**
- Modify: `python/main.py` (lines 85-87)

**Step 1: Add OTA flag clearing and auto-check to main.py**

Replace the web server startup section in `main.py` (lines 85-87):

```python
# Start web server in background thread (keeps REPL available)
_thread.stack_size(32768)
_thread.start_new_thread(web_server.start, ())
```

With:

```python
# Clear OTA file-update watchdog (boot succeeded)
try:
    import ota

    ota.clear_update_flag()
    ota.mark_firmware_valid()
    _boot_log("OTA: firmware valid, file update flag cleared")
except Exception as e:
    _boot_log("OTA init: " + str(e))


# Start web server with OTA auto-check after WiFi connects
def _start_with_ota():
    web_server.start()  # blocks after WiFi connect + server start
    # Note: start() blocks forever, so auto-check runs in web_server.start()
    # We'll add the auto-check call inside web_server.start() instead


_thread.stack_size(32768)
_thread.start_new_thread(web_server.start, ())
```

Actually, the auto-check needs to happen after WiFi connects. Modify `web_server.py`'s `start()` function (line 151-155) instead:

In `web_server.py`, replace the `start()` function:
```python
def start():
    """Connect WiFi and start the web server (runs forever)."""
    wifi.connect()
    print("Starting web server on http://" + wifi.ip + ":80")

    # OTA auto-check after WiFi connects
    if wifi.mode == "sta":
        try:
            cfg = ota.load_config()
            if cfg.get("auto_check", True):
                result = ota.check_for_updates()
                if result and (result["firmware_available"] or result["files_available"]):
                    _boot_log = lambda msg: (print("[BOOT]", msg), open("/boot.log", "a").write(msg + "\n"))
                    parts = []
                    if result["firmware_available"]:
                        parts.append("firmware " + result["latest_firmware"])
                    if result["files_available"]:
                        parts.append("files " + result["latest_files"])
                    print("[BOOT] OTA auto-check: update available (" + ", ".join(parts) + ")")
                    try:
                        with open("/boot.log", "a") as f:
                            f.write(
                                "OTA auto-check: update available ("
                                + ", ".join(parts)
                                + ")\n"
                            )
                    except Exception:
                        pass
        except Exception as e:
            print("[BOOT] OTA auto-check failed:", e)

    asyncio.run(app.start_server(host="0.0.0.0", port=80))
```

In `main.py`, the OTA flag clearing stays simple:
```python
# Clear OTA file-update watchdog (boot succeeded)
try:
    import ota

    ota.clear_update_flag()
    ota.mark_firmware_valid()
    _boot_log("OTA: firmware valid, update flag cleared")
except Exception as e:
    _boot_log("OTA init: " + str(e))

# Start web server in background thread (keeps REPL available)
_thread.stack_size(32768)
_thread.start_new_thread(web_server.start, ())
```

**Step 2: Verify boot sequence**

Flash updated firmware, reboot. Check REPL output for:
```
[BOOT] Firmware: 0.1.0, Files: unknown
[BOOT] DRV2665: init OK, reg2=0x40
[BOOT] OTA: firmware valid, update flag cleared
```

And check `/boot.log` contains all entries.

**Step 3: Commit**

```bash
git add python/main.py python/web_server.py
git commit -m "feat: integrate OTA in boot sequence with auto-check and watchdog"
```

---

### Task 8: Build, Flash, and End-to-End Test

**Step 1: Full build with OTA partition table**

```bash
cd ~/projects/optacon-firmware
./scripts/build.sh
ls -la build/micropython.bin  # verify < 1.5 MB
```

**Step 2: Flash firmware**

Use the flash skill to flash the board.

**Step 3: Upload filesystem files**

Upload the new/modified filesystem files to the board:
- `ota.py`
- `web_server.py`
- `web/update.html`
- `web/index.html` (updated footer)
- `web/wifi.html` (updated footer)
- `web/docs.html` (updated footer)

**Step 4: Verify boot log**

Check REPL output and read `/boot.log`:
```python
with open("/boot.log") as f:
    print(f.read())
```

Expected entries:
```
Firmware: 0.1.0, Files: unknown
DRV2665: init OK, reg2=0x40
OTA: firmware valid, update flag cleared
```

**Step 5: Verify web UI**

Navigate to `http://esp-optacon.local/update` in browser:
- Status card shows firmware and files version
- Log viewer shows boot.log content
- Settings form works
- Version footer appears on all pages (index, wifi, docs, update)

**Step 6: Verify OTA check (with test server)**

Set up a simple test server with versions.json to verify the check endpoint:
```python
# From REPL:
import ota
ota.save_config({"update_url": "http://192.168.x.x:8000/", "auto_check": True, "firmware_version": "0.1.0", "files_version": "0.1.0"})
result = ota.check_for_updates()
print(result)
```

**Step 7: Verify rollback mechanism**

Test the NVS watchdog manually:
```python
import esp32
nvs = esp32.NVS("ota")
nvs.set_i32("file_upd", 1)
nvs.commit()
# Create a fake .bak file
with open("/test.txt.bak", "w") as f:
    f.write("rollback test")
import machine
machine.soft_reset()
```

After reboot, check boot.log for rollback messages and verify `.bak` file was restored.

**Step 8: Final commit**

```bash
git add -A
git commit -m "feat: OTA firmware updates — complete implementation"
```
