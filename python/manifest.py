# Standard MicroPython frozen modules
include("$(PORT_DIR)/boards/manifest.py")

# Core drivers (frozen into firmware — rarely change)
freeze(".", ("boot.py", "pz_drive_py.py", "drv2665.py", "shift_register.py", "main.py"))

# Third-party (frozen — never changes)
package("microdot", base_path=".")
