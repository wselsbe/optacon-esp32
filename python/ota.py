"""OTA update manager — check, download, verify, apply updates."""
import hashlib
import json
import os

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


def _ver_gt(a, b):
    """Return True if version string a > b (semver-style comparison)."""
    try:
        at = tuple(int(x) for x in a.split("."))
        bt = tuple(int(x) for x in b.split("."))
        return at > bt
    except Exception:
        return a != b


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


def _device_headers():
    """Get X-Device-* headers for OTA requests."""
    try:
        import hw_info

        return hw_info.get_headers()
    except Exception:
        return {}


def _parse_url(url):
    """Parse URL into (host, port, path, use_ssl)."""
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
    return host, port, path, use_ssl


def _http_request(method, url, body=None, headers=None):
    """Minimal HTTP request returning (status, headers_dict, socket).

    Returns the raw socket for streaming. Caller must close it.
    """
    import socket

    try:
        import ssl
    except ImportError:
        ssl = None

    host, port, path, use_ssl = _parse_url(url)

    # Connect - wrap in try/except for socket safety
    addr = socket.getaddrinfo(host, port)[0][-1]
    s = socket.socket()
    try:
        s.settimeout(30)
        s.connect(addr)
        if use_ssl and ssl:
            s = ssl.wrap_socket(s, server_hostname=host)

        # Merge device identity headers
        all_headers = _device_headers()
        if headers:
            all_headers.update(headers)

        # Send request
        req = f"{method} {path} HTTP/1.0\r\nHost: {host}\r\n"
        for k, v in all_headers.items():
            req += f"{k}: {v}\r\n"
        if body is not None:
            req += f"Content-Length: {len(body)}\r\n"
        req += "\r\n"
        s.write(req.encode())
        if body is not None:
            s.write(body.encode() if isinstance(body, str) else body)

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
    except Exception:
        s.close()
        raise


def _http_get(url, headers=None):
    """Minimal HTTP GET returning (status, headers_dict, socket).

    Returns the raw socket for streaming. Caller must close it.
    """
    return _http_request("GET", url, headers=headers)


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
    sock = None
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
        sock = None  # mark as closed

        # Extract server date from headers
        date_str = headers.get("date", "")
        manifest = json.loads(data)
    except Exception as e:
        if sock:
            try:
                sock.close()
            except Exception:
                pass
        _log_check("Check FAILED — " + str(e))
        return None

    latest_fw = manifest.get("latest_firmware", "")
    latest_files = manifest.get("latest_files", "")
    current_fw = cfg.get("firmware_version", "0.0.0")
    current_files = cfg.get("files_version", "0.0.0")

    fw_available = bool(latest_fw) and _ver_gt(latest_fw, current_fw)
    files_available = bool(latest_files) and _ver_gt(latest_files, current_files)

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
        part_size = nxt.info()[3]  # (type, subtype, addr, size, label, encrypted)
        _log_update("Writing to partition: " + nxt.info()[4] + " (size=" + str(part_size) + ")")

        # Stream to partition
        sha = hashlib.sha256()
        block = 0
        total = 0
        while True:
            chunk = sock.read(4096)
            if not chunk:
                break
            total += len(chunk)
            if total > part_size:
                _log_update("Firmware too large: " + str(total) + " > " + str(part_size))
                sock.close()
                return False
            # Pad last block to 4096 if needed
            if len(chunk) < 4096:
                padded = chunk + b"\xff" * (4096 - len(chunk))
                nxt.writeblocks(block, padded)
            else:
                nxt.writeblocks(block, chunk)
            sha.update(chunk)
            block += 1
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

    # Record updated paths for rollback/cleanup
    with open("/ota_pending.json", "w") as f:
        json.dump(downloaded, f)

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
        # Clean up .bak files using the recorded paths
        try:
            with open("/ota_pending.json") as f:
                paths = json.load(f)
            for path in paths:
                try:
                    os.remove(path + ".bak")
                except OSError:
                    pass
            os.remove("/ota_pending.json")
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
    import boot_cfg

    return {
        "firmware_version": boot_cfg.FIRMWARE_VERSION,
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

    import boot_cfg

    cfg = load_config()
    url = cfg.get("diagnostics_url", "")
    if not url:
        return False

    diag = {
        "firmware_version": boot_cfg.FIRMWARE_VERSION,
        "files_version": cfg.get("files_version", "unknown"),
        "free_mem": gc.mem_free(),
        "boot_log": get_log("boot"),
        "boot_log_prev": get_log("boot.prev"),
        "check_log": get_log("check"),
        "update_log": get_log("update"),
    }

    try:
        body = json.dumps(diag)
        hdrs = {"Content-Type": "application/json"}
        status, _resp_hdrs, sock = _http_request("POST", url, body=body, headers=hdrs)
        sock.close()

        _log_update("Diagnostics sent — HTTP " + str(status))
        return status == 200

    except Exception as e:
        _log_update("Diagnostics send FAILED — " + str(e))
        return False
