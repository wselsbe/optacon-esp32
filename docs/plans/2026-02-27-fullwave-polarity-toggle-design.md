# Full-Wave Polarity Toggle Design

## Problem

The DRV2665 single-ended output delivers ~100V pk-pk max. To achieve ~200V pk-pk across the actuator, we toggle the HV509 polarity at each sine zero-crossing, effectively creating a full-wave drive.

## Concept

Generate |sin(2*pi*f*t)| on the PWM output (a full-wave rectified sine). Toggle polarity GPIO12/13 at each zero-crossing. From the actuator's perspective, this reconstructs a full sine at frequency f with double the effective voltage swing.

For requested frequency f:
- DDS runs at f Hz
- ISR outputs the rectified (absolute value) waveform
- ISR toggles polarity at each zero-crossing (when phase accumulator bit 31 flips)

## Approach: Phase-Aware Rectification in ISR

The DDS phase accumulator's top bit (bit 31) divides the cycle into two halves:
- **First half** (bit 31 = 0, phase 0 to pi): LUT values go 128 -> 255 -> 128. Output as-is.
- **Second half** (bit 31 = 1, phase pi to 2*pi): LUT values go 128 -> 0 -> 128. Mirror around 128: `duty = 256 - lut_value`.

Polarity toggles exactly when bit 31 changes — perfectly synchronized, zero jitter.

## Changes

### C module: `pz_pwm.c`

**Polarity GPIO ownership** (moved from Python ShiftRegister):
- GPIO 12 and 13 configured as outputs at **module init time** (before SPI init, avoiding IOMUX conflict)
- Initial state: both LOW (inverted mode = normal operation)
- New functions: `pz_pwm.set_polarity(value)`, `pz_pwm.get_polarity()`

**Fullwave mode**:
- New state: `s_fullwave` (bool), `s_prev_half` (uint8_t)
- `set_frequency()` gets `fullwave` kwarg (default 0)
- ISR changes when `s_fullwave` is true:
  - Check `half = (s_phase_acc >> 31) & 1`
  - If `half == 1`: `duty = 256 - lut_value` (mirror negative half)
  - If `half != s_prev_half`: toggle GPIO12/13, update `s_prev_half`
- `start()`: reset polarity GPIOs to LOW, reset `s_prev_half` to 0
- `stop()`: reset polarity GPIOs to LOW

### Python: `shift_register.py`

- Remove `_pol_a`, `_pol_b`, `_polarity` fields
- Remove `toggle_polarity()` and `get_polarity()` methods
- Remove `pol_a_pin`/`pol_b_pin` constructor params

### Python: `pz_actuator_py.py`

- `set_frequency_analog()`: add `fullwave=False` param, pass to `pz_pwm.set_frequency()`
- `toggle_polarity()`: call `pz_pwm.set_polarity(not pz_pwm.get_polarity())`
- `get_polarity()`: call `pz_pwm.get_polarity()`
- `stop()`: polarity reset handled by C module

### Python: `main.py`

- Update any demo code that uses polarity functions

## Scope

- Analog mode only; digital mode is out of scope
- No changes to shift register pin mapping or DRV2665 driver
