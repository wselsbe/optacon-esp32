"""Read-only device identity from hw_info NVS partition.

Written once per board via scripts/provision.py. Never modified by firmware
or OTA updates. All values are sent as X-Device-* HTTP headers during OTA.
"""

try:
    import esp32
    _nvs = esp32.NVS("hw_info")
except Exception:
    _nvs = None

_KEYS = ("serial_number", "hw_revision")


def get(key, default=None):
    """Read a string value from the hw_info partition."""
    if _nvs is None:
        return default
    try:
        buf = bytearray(64)
        _nvs.get_blob(key, buf)
        return buf.rstrip(b"\x00").decode()
    except OSError:
        return default


def get_all():
    """Return dict of all known device identity keys."""
    return {k: get(k) for k in _KEYS if get(k) is not None}


def get_headers():
    """Return dict of X-Device-* HTTP headers for OTA requests."""
    import boot_cfg

    headers = {}
    for k, v in get_all().items():
        # serial_number -> X-Device-Serial-Number
        header_name = "X-Device-" + k.replace("_", "-").title()
        headers[header_name] = v
    headers["X-Device-Firmware-Version"] = boot_cfg.FIRMWARE_VERSION
    return headers
