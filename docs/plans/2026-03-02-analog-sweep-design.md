# Analog Frequency Sweep Design

## Goal

Add frequency sweep to analog waveform generation. Sweep a sine/triangle/square wave between two frequencies over a configurable duration, with linear or logarithmic mode. Non-fullwave only. Output holds the end frequency indefinitely after the sweep completes.

## Approach: ISR-Level Sweep

Extend the existing 32 kHz DDS ISR to modulate `s_phase_step` each tick. Zero RAM overhead, perfectly smooth transitions.

## C Layer (`pwm.c`)

### New State Variables

```c
static volatile bool     s_sweep_active = false;
static volatile uint32_t s_phase_step_target = 0;
static volatile bool     s_sweep_linear = true;
// Linear mode:
static volatile int32_t  s_sweep_delta = 0;       // added to phase_step per tick
// Log mode:
static volatile uint32_t s_sweep_ratio = 0;        // fixed-point 1.31 multiplier
static volatile bool     s_sweep_up = true;         // true = freq increasing
```

### ISR Modification

After `s_phase_acc += s_phase_step`, before waveform lookup:

```c
if (s_sweep_active) {
    if (s_sweep_linear) {
        int32_t next = (int32_t)s_phase_step + s_sweep_delta;
        s_phase_step = (uint32_t)next;
    } else {
        s_phase_step = (uint32_t)(((uint64_t)s_phase_step * s_sweep_ratio) >> 31);
    }
    // Clamp at target
    if (s_sweep_up ? (s_phase_step >= s_phase_step_target)
                   : (s_phase_step <= s_phase_step_target)) {
        s_phase_step = s_phase_step_target;
        s_sweep_active = false;
    }
}
```

### New C Function

```c
void pzd_pwm_set_sweep(int start_hz, int end_hz, int duration_ms, bool logarithmic);
```

Computes:
- `s_phase_step` from `start_hz`
- `s_phase_step_target` from `end_hz`
- `s_sweep_up` = `end_hz > start_hz`
- Linear: `s_sweep_delta = (target - start) / total_ticks` where `total_ticks = duration_ms * 32`
- Log: `s_sweep_ratio = (end/start)^(1/total_ticks)` in 1.31 fixed-point

Sets `s_sweep_active = true`. Also sets `s_freq_configured = true` so `pwm_start()` works.

### Log Sweep Ratio Computation

Need `ratio = (f_end/f_start)^(1/N)` where N = total ISR ticks.

Use integer approximation: `ratio ≈ 1 + ln(f_end/f_start) / N` in 1.31 fixed-point. For small per-tick changes this is accurate. In 1.31 format, 1.0 = 0x80000000, so `ratio = 0x80000000 + (ln(f_end/f_start) * 0x80000000) / N`.

Compute `ln(f_end/f_start)` using integer log2 approximation or pass the ratio from Python (which has `math.log`).

**Decision**: Compute the ratio in Python and pass it to C as a fixed-point integer, avoiding the need for floating-point math in C.

## Python Binding (`pz_drive.c`)

```python
pz_drive.pwm_set_sweep(start_hz, end_hz, duration_ms, logarithmic=False)
# logarithmic: if False, pass sweep_delta (int32); if True, pass sweep_ratio (uint32 fixed-point)
```

Alternative: have Python compute the per-tick delta/ratio and pass it directly:

```python
pz_drive.pwm_start_sweep(start_hz, end_hz, delta_or_ratio, is_log)
```

This keeps the C code simple (no float math, no log computation).

## Python Layer (`pz_drive_py.py`)

```python
def sweep_analog(self, start_hz, end_hz, duration_ms,
                 logarithmic=False, waveform='sine',
                 resolution=8, amplitude=100):
    """Sweep frequency from start_hz to end_hz over duration_ms.

    After the sweep completes, output holds at end_hz until stop().
    """
```

Implementation:
1. Configure waveform via `pwm_set_frequency(start_hz, ...)`
2. Compute sweep parameters in Python (delta for linear, ratio for log)
3. Call `pz_drive.pwm_set_sweep(...)` with computed parameters
4. Init DRV2665 analog mode and call `pz_drive.pwm_start()`

## Waveform Interaction

The sweep only modulates frequency (phase_step). The waveform shape (sine/triangle/square), amplitude, and resolution remain unchanged throughout the sweep.

## Shift Register Latching

Non-fullwave latching at cycle-wrap continues to work. As frequency changes, the latch rate changes proportionally (latches once per waveform cycle).

## Constraints

- Non-fullwave only (no polarity toggling during sweep)
- Analog path only (not digital FIFO)
- Frequency range: 1-500 Hz for both start and end
- Duration: 1 ms to 60,000 ms (1 minute)
