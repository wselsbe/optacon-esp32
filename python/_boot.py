"""VFS mount + stock boot.py cleanup.

This replaces MicroPython's default _boot.py (ports/esp32/modules/_boot.py).
The VFS mount and inisetup logic is identical to the vendor version.

The addition: after inisetup formats a fresh filesystem, it creates a stock
boot.py with commented-out webrepl/osdebug stubs.  That filesystem boot.py
shadows our frozen boot.py, which handles OTA rollback safety and boot
logging.  We remove it immediately so the frozen version always runs.

This only matters on first boot after a fresh flash (factory provisioning).
OTA updates don't touch the filesystem, so it won't recur.
"""

import gc
import os
import vfs

from flashbdev import bdev

try:
    if bdev:
        vfs.mount(bdev, "/")
except OSError:
    import inisetup

    inisetup.setup()

# Remove the stock filesystem boot.py that inisetup.setup() creates.
# Our frozen boot.py provides OTA rollback safety and must not be shadowed.
try:
    os.remove("/boot.py")
except OSError:
    pass

gc.collect()
