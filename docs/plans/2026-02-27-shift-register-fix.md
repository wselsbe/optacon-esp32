# Shift Register (HV509) Fix — SPI & Polarity

**Date:** 2026-02-27

## Summary

Fixed two issues preventing the HV509 shift registers from working: an SPI2 IOMUX conflict and incorrect polarity pin assignment.

## Issues Fixed

### 1. Wrong Polarity GPIO

The polarity control was on GPIO34. The correct pins are **GPIO12 and GPIO13** (both driven to the same state). Updated `shift_register.h` to define `SHIFTREG_POLARITY_PIN_A` (12) and `SHIFTREG_POLARITY_PIN_B` (13).

### 2. SPI2 IOMUX Conflict

GPIO10–13 are the default IOMUX pins for SPI2_HOST on ESP32-S3. Configuring GPIO12/13 as GPIO outputs **after** `spi_bus_initialize()` disrupted the SPI peripheral — CS (GPIO10) stopped toggling despite `spi_device_polling_transmit()` returning ESP_OK.

**Fix:** Moved polarity GPIO configuration to **before** the SPI bus initialization in `shift_register_init()`.

### 3. Polarity Default Value

The correct default is `polarity = true` (GPIO HIGH → POL_bar = HIGH → BP = LOW = 0V). With this setting:

- BP stays at ~0V (measured 18mV mean, 2.4V pk-pk coupling noise)
- `set_all(True)` → actuator output = 95.2V pk-pk sine wave (ON)
- `set_all(False)` → actuator output = 2.4V pk-pk (OFF)

The DRV2665 OUT+ connects to HV509 VPP. With POL_bar=HIGH, BP=LOW and enabled outputs are driven to VPP (the high-voltage sine wave). Actuators are connected between individual HV509 outputs and BP (ground reference).

## Verification

- **BP at 0V confirmed** with scope (CH1) after init, before starting DRV2665
- **SPI control confirmed**: set_all(True) → 95.2V on actuator, set_all(False) → 2.4V
- **Individual pin control confirmed**: set_pin(4, True) enables only that actuator
- **Actuator physically moving** observed during pin_cycle test

## Open Issue: Only One Pin Actuating

During the 20-pin cycle test (`pin_cycle.py`), only a single actuator was observed physically moving, despite the SPI data changing for all 20 pins and scope confirming voltage on the probed pin.

Possible causes:
- Other actuators may not be properly connected or soldered
- HV509 output drivers may have an issue on certain channels
- SPI data may not be latching correctly for all bit positions

**Recommended next step:** Retest with the logic analyzer connected to SPI lines (MOSI, SCK, CS) to verify the full 32-bit shift register data is being clocked out correctly and latched. Compare the SPI transaction for different pin selections to confirm all 20 bit positions are being set.

## Files Changed

- `modules/pz_actuator/shift_register.h` — Polarity pin defines (GPIO12/13), updated comment
- `modules/pz_actuator/shift_register.c` — Init order fix (GPIO before SPI), default polarity=true, removed debug prints, removed unused `py/mpprint.h` include
- `CLAUDE.md` — Updated polarity pin table (12/13 instead of 34)
