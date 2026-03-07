"""Install fake MicroPython-specific modules into sys.modules."""

import sys
import time
import types
from unittest.mock import MagicMock

# ---------------------------------------------------------------------------
# Patch MicroPython time helpers onto the real time module
# ---------------------------------------------------------------------------

if not hasattr(time, "sleep_ms"):
    time.sleep_ms = lambda ms: time.sleep(ms / 1000)

if not hasattr(time, "ticks_ms"):
    time.ticks_ms = lambda: int(time.time() * 1000) & 0x3FFFFFFF

if not hasattr(time, "ticks_diff"):
    time.ticks_diff = lambda a, b: (a - b) & 0x3FFFFFFF


# ---------------------------------------------------------------------------
# Helper to create a simple module and register it
# ---------------------------------------------------------------------------

def _make_module(name, attrs):
    """Create a fake module with the given attributes and register it."""
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    sys.modules[name] = mod
    return mod


# ---------------------------------------------------------------------------
# machine
# ---------------------------------------------------------------------------

_machine_Pin = MagicMock(name="machine.Pin")
_machine_Pin.OUT = 1
_machine_Pin.IN = 0
_machine_Pin.PULL_UP = 1

_machine_I2C = MagicMock(name="machine.I2C")
_machine_SPI = MagicMock(name="machine.SPI")
_machine_Timer = MagicMock(name="machine.Timer")
_machine_reset = MagicMock(name="machine.reset")
_machine_freq = MagicMock(name="machine.freq", return_value=240000000)

_machine = _make_module("machine", {
    "Pin": _machine_Pin,
    "I2C": _machine_I2C,
    "SPI": _machine_SPI,
    "Timer": _machine_Timer,
    "reset": _machine_reset,
    "freq": _machine_freq,
})

# ---------------------------------------------------------------------------
# esp32
# ---------------------------------------------------------------------------

_esp32_NVS = MagicMock(name="esp32.NVS")
_esp32_NVS.return_value = _esp32_NVS  # calling NVS() returns itself

_esp32_Partition = MagicMock(name="esp32.Partition")
_esp32_Partition.RUNNING = 0
_next_update = MagicMock(name="esp32.Partition.next_update")
_next_update.info.return_value = (0, 0, 0, 1572864, "ota_1", False)
_esp32_Partition.return_value = _esp32_Partition
_esp32_Partition.get_next_update = MagicMock(return_value=_next_update)

_esp32 = _make_module("esp32", {
    "NVS": _esp32_NVS,
    "Partition": _esp32_Partition,
})

# ---------------------------------------------------------------------------
# network
# ---------------------------------------------------------------------------

class MockWLAN:
    """WLAN mock with state tracking. Methods are MagicMocks so tests can
    override ``return_value`` (e.g. ``_wlan_instance.isconnected.return_value = False``)."""

    STA_IF = 0
    AP_IF = 1

    def __init__(self, mode):
        self._active = False
        self.active = MagicMock(side_effect=self._active_fn)
        self.connect = MagicMock()
        self.disconnect = MagicMock(side_effect=self._disconnect_fn)
        self.ifconfig = MagicMock(
            return_value=("192.168.1.100", "255.255.255.0", "192.168.1.1", "8.8.8.8")
        )
        self.config = MagicMock(return_value="test_ssid")
        self.status = MagicMock(return_value=0)
        # isconnected uses return_value (no side_effect) so tests can override
        # it directly via ``wlan.isconnected.return_value = True/False``.
        # disconnect() automatically sets return_value to False.
        self.isconnected = MagicMock(return_value=False)

    def _active_fn(self, val=None):
        if val is not None:
            self._active = val
        return self._active

    def _disconnect_fn(self):
        self.isconnected.return_value = False


_network_WLAN = MagicMock(name="network.WLAN")
_wlan_instance = MockWLAN(0)
# Default: connected (matches previous mock default for existing tests)
_wlan_instance._connected = True
_wlan_instance.isconnected.return_value = True
_network_WLAN.return_value = _wlan_instance
_network_hostname = MagicMock(name="network.hostname")

_network = _make_module("network", {
    "WLAN": _network_WLAN,
    "STA_IF": 0,
    "AP_IF": 1,
    "hostname": _network_hostname,
})

# ---------------------------------------------------------------------------
# ntptime
# ---------------------------------------------------------------------------

_ntptime_settime = MagicMock(name="ntptime.settime")

_ntptime = _make_module("ntptime", {
    "settime": _ntptime_settime,
})

# ---------------------------------------------------------------------------
# neopixel
# ---------------------------------------------------------------------------

_neopixel_NeoPixel = MagicMock(name="neopixel.NeoPixel")

_neopixel = _make_module("neopixel", {
    "NeoPixel": _neopixel_NeoPixel,
})

# ---------------------------------------------------------------------------
# _thread
# ---------------------------------------------------------------------------

_thread_start_new_thread = MagicMock(name="_thread.start_new_thread")

_thread = _make_module("_thread", {
    "start_new_thread": _thread_start_new_thread,
})

# ---------------------------------------------------------------------------
# boot_cfg
# ---------------------------------------------------------------------------

_boot_cfg_log = MagicMock(name="boot_cfg._log")

_boot_cfg = _make_module("boot_cfg", {
    "FIRMWARE_VERSION": "0.1.0",
    "_LOG": "/boot.log",
    "_LOG_PREV": "/boot.log.prev",
    "_log": _boot_cfg_log,
})

# ---------------------------------------------------------------------------
# webrepl / webrepl_cfg
# ---------------------------------------------------------------------------

_webrepl = _make_module("webrepl", {
    "start": MagicMock(name="webrepl.start"),
})

_webrepl_cfg = _make_module("webrepl_cfg", {
    "PASS": "test",
})


# ---------------------------------------------------------------------------
# Reset all mocks
# ---------------------------------------------------------------------------

def _reset_all():
    """Reset all mock modules and restore defaults."""
    # machine
    _machine_Pin.reset_mock()
    _machine_Pin.OUT = 1
    _machine_Pin.IN = 0
    _machine_Pin.PULL_UP = 1
    _machine_I2C.reset_mock()
    _machine_SPI.reset_mock()
    _machine_Timer.reset_mock()
    _machine_reset.reset_mock()
    _machine_freq.reset_mock(return_value=True)
    _machine_freq.return_value = 240000000

    # esp32
    _esp32_NVS.reset_mock()
    _esp32_NVS.return_value = _esp32_NVS
    _esp32_Partition.reset_mock()
    _esp32_Partition.RUNNING = 0
    _next_update.reset_mock()
    _next_update.info.return_value = (0, 0, 0, 1572864, "ota_1", False)
    _esp32_Partition.return_value = _esp32_Partition
    _esp32_Partition.get_next_update = MagicMock(return_value=_next_update)

    # network
    global _wlan_instance
    _network_WLAN.reset_mock()
    _wlan_instance = MockWLAN(0)
    _wlan_instance._connected = True
    _wlan_instance.isconnected.return_value = True
    _network_WLAN.return_value = _wlan_instance
    _network_hostname.reset_mock()

    # ntptime
    _ntptime_settime.reset_mock()

    # neopixel
    _neopixel_NeoPixel.reset_mock()

    # _thread
    _thread_start_new_thread.reset_mock()

    # boot_cfg
    _boot_cfg.FIRMWARE_VERSION = "0.1.0"
    _boot_cfg._LOG = "/boot.log"
    _boot_cfg._LOG_PREV = "/boot.log.prev"
    _boot_cfg_log.reset_mock()
    _boot_cfg._log = _boot_cfg_log

    # webrepl
    _webrepl.start.reset_mock()
