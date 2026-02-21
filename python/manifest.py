# Standard MicroPython frozen modules
include("$(PORT_DIR)/boards/manifest.py")

# Application modules
freeze("/workspace/python")
