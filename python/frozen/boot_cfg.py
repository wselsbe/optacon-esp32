"""Shared boot constants and logging (importable frozen module)."""

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
