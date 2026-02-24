BOARD           ?= ESP32_GENERIC_S3
PORT            ?= COM7
MICROPYTHON_DIR ?= /opt/micropython

MPY_PORT_DIR     = $(MICROPYTHON_DIR)/ports/esp32
BUILD_DIR        = $(MPY_PORT_DIR)/build-$(BOARD)

# mpy-cross: .exe on Windows, plain binary in container
ifeq ($(OS),Windows_NT)
export MICROPY_MPYCROSS ?= $(MICROPYTHON_DIR)/mpy-cross/build/mpy-cross.exe
else
export MICROPY_MPYCROSS ?= $(MICROPYTHON_DIR)/mpy-cross/build/mpy-cross
endif

# ── Targets ──────────────────────────────────────────────────────────
.PHONY: build flash clean repl submodules

build: submodules
	cd "$(MPY_PORT_DIR)" && idf.py \
		-D MICROPY_BOARD=$(BOARD) \
		-D MICROPY_BOARD_DIR=$(MPY_PORT_DIR)/boards/$(BOARD) \
		-D USER_C_MODULES=$(CURDIR)/modules/micropython.cmake \
		-D MICROPY_FROZEN_MANIFEST=$(CURDIR)/python/manifest.py \
		-B build-$(BOARD) build

# Flash from host — binaries are in build-cache/ via docker-compose volume
FLASH_DIR ?= build-cache
flash:
	python -m esptool --chip esp32s3 \
		-p $(PORT) -b 460800 \
		--before default_reset --after hard_reset \
		write_flash --flash_mode dio --flash_size 4MB --flash_freq 80m \
		0x0     $(FLASH_DIR)/bootloader/bootloader.bin \
		0x8000  $(FLASH_DIR)/partition_table/partition-table.bin \
		0x10000 $(FLASH_DIR)/micropython.bin

clean:
	-rm -rf "$(BUILD_DIR)"

repl:
	mpremote connect $(PORT)

submodules:
	cd "$(MPY_PORT_DIR)" && idf.py \
		-D MICROPY_BOARD=$(BOARD) \
		-D MICROPY_BOARD_DIR=$(MPY_PORT_DIR)/boards/$(BOARD) \
		-B build-$(BOARD)/submodules -D UPDATE_SUBMODULES=1 reconfigure
