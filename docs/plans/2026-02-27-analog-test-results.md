# Analog Path Test Results

**Date:** 2026-02-27
**Branch:** main
**Firmware:** API refactor build (set_frequency_analog/digital, start/stop with gain kwarg)

## Summary

Tested the analog sine wave path (Phase 0 and Phase 1 from the test plan). **After replacing C9 with correct 100nF capacitor, the full analog path works end-to-end.** Frequency sweep, DC mode, 10-bit resolution, and gain sweep all pass.

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
- 8-bit mode: 312.5 kHz carrier, confirmed 250 Hz envelope
- 10-bit mode: 76.3 kHz carrier, confirmed 250 Hz envelope
- DC mode (freq=0): flat 100% duty at 3.30V mean — correct
- Mean voltage ~1.69V (mid-rail) — correct for unipolar PWM sine

### Analog path end-to-end — PASS (after C9 fix)

C9 replaced with confirmed 100nF (104) 0603 ceramic. Full signal chain now works:
GPIO5 PWM → R13+C9 filter → C3 AC coupling → DRV2665 IN+ → boost converter → actuator

**Filtered output (CH4, after R13+C9):** 3.44V pk-pk, 1.69V mean — clean sine
**IN+ (CH3, after C3):** 3.44V pk-pk — AC-coupled sine at DRV2665 bias point
**VPP (CH1, DRV2665 HV output):** ~52V mean with sine modulation at gain=100

### Frequency sweep — PASS

All frequencies within ~1% of target. Amplitude rock-steady at 3.44V pk-pk across range.

| Target | Measured (CH4) | Pk-Pk (CH4) | Error |
|--------|----------------|-------------|-------|
| 50 Hz  | 50.2 Hz        | 3.44V       | +0.4% |
| 100 Hz | 100.6 Hz       | 3.44V       | +0.6% |
| 200 Hz | 201.3 Hz       | 3.44V       | +0.6% |
| 250 Hz | 250.8 Hz       | 3.44V       | +0.3% |
| 300 Hz | 301.9 Hz       | 3.44V       | +0.6% |
| 400 Hz | 401.2 Hz       | 3.44V       | +0.3% |

### DC mode (freq=0) — PASS
- CH4 (filtered): 3.30V mean (100% duty), 200mV pk-pk ripple
- CH1 (VPP): 51.96V — boost active at DC

### 10-bit resolution — PASS
- 250 Hz at 10-bit: 250.41 Hz, 3.44V pk-pk, 1.70V mean
- Visibly smoother sine compared to 8-bit (lower carrier freq = 76.3 kHz)

### Gain sweep — PASS

Gain register correctly applied in analog mode. Datasheet confirms gain is dB-based for analog input.

| Gain | CTRL1 | Analog dB | VPP pk-pk | Notes |
|------|-------|-----------|-----------|-------|
| 25V  | 0x04  | 28.8 dB   | 46.4V     | Clean sine, no clipping |
| 50V  | 0x05  | 34.8 dB   | 92.0V     | Clean sine, no clipping |
| 75V  | 0x06  | 38.4 dB   | 104.8V    | Clipping at VBST (~105V) |
| 100V | 0x07  | 40.7 dB   | 104.8V    | Heavy clipping at VBST |

- 25→50V gain ratio: 92/46.4 = 1.98x (~6 dB) — matches datasheet
- At gain=75 and 100, output clips at VBST ceiling (~105V), producing trapezoidal waveform
- Gain=25 with 3.44V input: 46.4V output → voltage gain ~27.5x → 28.8 dB ✓

### DRV2665 register configuration — PASS
- Init sequence follows datasheet 8.3.1 (removed reset, correct register order)
- After `start(gain=N)`:
  - CTRL1 correctly set: 0x04 (gain=25), 0x05 (50), 0x06 (75), 0x07 (100)
  - CTRL2 = 0x0E — EN_OVERRIDE + timeout 20ms ✓

## Hardware Fix Applied

### C9 replaced (2026-02-27)
- **Was:** wrong value (likely 100pF / "101" marking) — filter cutoff ~4 MHz, no filtering
- **Now:** 100nF (104) 0603 ceramic — filter cutoff = 1/(2π × 390 × 100nF) = 4.08 kHz
- Result: clean filtered sine reaches DRV2665 IN+, boost converter activates, full HV output

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

## Next Steps

1. Phase 2: Digital FIFO path testing (frequency sweep, FIFO timing)
2. Phase 3: Mode switching (analog↔digital transitions)
3. Phase 4: Pin control (shift register, individual actuator channels)
4. Phase 5: Error handling and edge cases
5. Consider C9=220nF to reduce carrier ripple further (currently ~480mV at 250Hz/8-bit)
6. For 6-board parallel use: R13=150Ω + C9=270nF recommended

## Scope/Probe Notes

- Probe mapping (updated 2026-02-27):
  - CH1 = VPP (DRV2665 high-voltage actuator output)
  - CH2 = VBST (boost converter)
  - CH3 = IN+ (after C3 coupling cap, at DRV2665 input)
  - CH4 = filtered PWM (after R13+C9 RC filter, before C3 coupling cap)
- Scope timebase via SCPI (`TDIV`) doesn't work when scope is in zoom mode — must use physical knob
- Frequency counter works on CH4 (filtered sine) but not on raw PWM (carrier confuses it)
