# Standard MicroPython frozen modules
include("$(PORT_DIR)/boards/manifest.py")

# Core boot + drivers (frozen into firmware — rarely change)
# _boot.py is overlaid onto vendor modules dir by build.sh (can't freeze twice)
freeze(".", ("boot_cfg.py", "boot.py", "pz_drive_py.py", "drv2665.py", "shift_register.py", "main.py"))

# Third-party (frozen — never changes)
package("microdot", base_path=".")
