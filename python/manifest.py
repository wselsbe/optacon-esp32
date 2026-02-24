# Standard MicroPython frozen modules
include("$(PORT_DIR)/boards/manifest.py")

# Application modules — freeze Python files from this directory
# (makemanifest.py chdir's to the manifest's directory before executing it)
freeze(".")
