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

    # Read list of updated files
    paths = []
    try:
        import json

        with open("/ota_pending.json") as f:
            paths = json.load(f)
    except Exception:
        # Fallback: scan root for .bak files (won't find subdirs)
        for name in os.listdir("/"):
            if name.endswith(".bak"):
                paths.append("/" + name[:-4])

    for path in paths:
        bak_path = path + ".bak"
        try:
            os.rename(bak_path, path)
            _log("Rollback: " + bak_path + " -> " + path)
            rolled += 1
        except OSError as e:
            _log("Rollback FAILED: " + bak_path + " " + str(e))

    # Clean up pending file
    try:
        os.remove("/ota_pending.json")
    except OSError:
        pass

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
del cfg

_check_rollback()
