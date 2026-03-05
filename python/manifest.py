# Standard MicroPython frozen modules
include("$(PORT_DIR)/boards/manifest.py")

# Core boot + drivers (frozen into firmware — rarely change)
# All .py files in frozen/ are auto-included; add new modules there.
# _boot.py is overlaid onto vendor modules dir by build.sh (can't freeze twice)
freeze("frozen")

# Third-party (frozen — never changes)
package("microdot", base_path=".")
