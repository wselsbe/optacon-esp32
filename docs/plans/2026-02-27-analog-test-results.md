# Analog Path Test Results

**Date:** 2026-02-27
**Branch:** main
**Firmware:** API refactor build (set_frequency_analog/digital, start/stop with gain kwarg)

## Summary

Tested the analog sine wave path (Phase 0 and Phase 1 from the test plan). **Digital mode works. Analog mode has a hardware issue: C9 (RC filter cap) is likely the wrong value, preventing the filtered sine from reaching DRV2665 IN+.**

## What Works

### Phase 0: Init — PASS
- `pz_actuator.init()` succeeds (I2C, SPI, PWM all initialize)
- `is_running()` returns `False`
- New API functions present: `set_frequency_analog`, `set_frequency_digital`, `read_reg`, `write_reg`

### Digital mode — PASS
- `set_frequency_digital(250)` + `start()` activates DRV2665 boost converter
- Current draw increases, VBST shows switching activity (~10V pk-pk on CH1)
- Actuator output visible on CH2

### PWM sine generation — PASS
- `set_frequency_analog(250)` + `start()` produces correct PWM on GPIO5
- CH4 (raw PWM) shows clear sine envelope modulating duty cycle 0%–100%
- 8-bit mode: 312.5 kHz carrier, confirmed 250 Hz envelope
- 10-bit mode: 76.3 kHz carrier, confirmed 250 Hz envelope
- DC mode (freq=0): flat 100% duty at 3.32V mean — correct
- Mean voltage 1.65V (mid-rail) — correct for unipolar PWM sine

### DRV2665 register configuration — PASS (after fix)
- **Fixed:** init sequence now follows datasheet 8.3.1 (removed reset, correct register order)
- After `start()` in analog mode:
  - CTRL1 (0x01) = 0x07 — analog input + gain 100V ✓
  - CTRL2 (0x02) = 0x0E — EN_OVERRIDE + timeout 20ms ✓
- Registers verified with new `read_reg()` debug function

## What Doesn't Work

### Analog path to DRV2665 — FAIL (hardware)

**Symptom:** DRV2665 boost converter does not activate in analog mode. Current stays at ~50mA (ESP32 + DRV2665 idle, no boost activity). No actuator output on CH2.

**Root cause:** The RC low-pass filter (R13 + C9) is not attenuating the PWM carrier.

**Evidence:**
- CH4 (raw PWM, before filter): beautiful 250 Hz sine envelope on 312 kHz carrier ✓
- CH3 (filtered, at IN+): only ~450mV pk-pk of carrier ripple at 2.46V DC, **no 250 Hz sine envelope**
- The 2.46V DC on CH3 is the DRV2665 internal IN+ bias (seen through AC coupling cap C3)
- In-circuit measurement of C9 shows **much less capacitance than expected 100nF**

**Likely cause:** C9 is wrong value — possibly 100pF (marking "101") instead of 100nF (marking "104"). This would shift the filter cutoff from 4 kHz to 4 MHz, providing essentially zero filtering.

## Code Changes Made

### drv2665.c — Init sequence fix
1. **Removed DRV2665_RESET** from `drv2665_init()` — was corrupting device state
2. **Fixed `drv2665_enable_analog()` register order** to match datasheet 8.3.1:
   - Step 1: Write CTRL2 to exit standby (clear STANDBY bit)
   - Step 2: Wait 5ms
   - Step 3: Write CTRL1 with analog mode + gain
   - Step 4: Write CTRL2 with EN_OVERRIDE + timeout (enables boost)
3. **Fixed `drv2665_enable_digital()` similarly** — exit standby first, then configure

### pz_actuator.c — Debug functions
- Added `read_reg(reg)` — read DRV2665 register via existing I2C bus handle
- Added `write_reg(reg, val)` — write DRV2665 register

## Hardware TODO

1. **Verify C9 value** — desolder and measure, or replace with known 100nF (104) 0603 ceramic
2. After C9 fix, re-run Phase 1 analog tests
3. Then proceed to Phase 2–6

## Scope/Probe Notes

- Scope timebase via SCPI (`TDIV`) doesn't work when scope is in zoom mode — must use physical knob
- Probe mapping (corrected from plan):
  - CH1 = VBST
  - CH2 = actuator output pin 4
  - CH3 = filtered PWM (after RC filter, at DRV2665 IN+)
  - CH4 = raw PWM (GPIO5, before R13)
