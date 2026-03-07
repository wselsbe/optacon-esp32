# Waveform Types Design

**Goal:** Add sine/triangle/square waveform selection to the analog DDS path.

**Approach:** Compute waveforms from the existing DDS phase accumulator in the 32 kHz ISR. Sine uses the existing 256-entry LUT. Triangle and square are computed directly from the 8-bit phase index — no extra LUTs.

## ISR Waveform Generation

All waveforms produce a centered value in -128..+127, then amplitude scaling and fullwave/dead_time/phase_advance apply unchanged.

- **Sine**: `raw = sine_lut[index] - 128` (existing)
- **Triangle**: `raw = (index < 128 ? index*2 : 510 - index*2) - 128`
- **Square**: `raw = (index < 128) ? 127 : -128`

Fullwave square = constant max output + polarity toggle (no special case needed).

## API

- C: `pz_pwm.set_frequency(hz, ..., waveform=0)` — 0=sine, 1=triangle, 2=square
- Python: `pa.set_frequency_analog(hz, ..., waveform='sine')` — string mapped to int

## Files

- `modules/pz_pwm/pz_pwm.c` — state var, ISR switch, kwarg
- `python/pz_actuator_py.py` — string-to-int mapping, pass-through
