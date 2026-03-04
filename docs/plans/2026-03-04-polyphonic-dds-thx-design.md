# Polyphonic DDS Mode — THX Deep Note

**Goal:** Add multi-voice DDS synthesis to the pz_drive C module, enabling THX Deep Note recreation where 12 voices converge from a random cluster to a D chord.

**Architecture:** New polyphonic ISR mode alongside existing mono DDS. Runs at 16kHz sample rate (vs 32kHz mono) to fit 12 voices in the timing budget. Each voice has independent phase accumulator, amplitude, and linear sweep. Python orchestrates the sequence; C handles the real-time math.

## C Module: Polyphonic ISR

### Per-voice state

```c
#define POLY_NUM_VOICES 12  // compile-time configurable

typedef struct {
    uint32_t phase_acc;
    uint32_t phase_step;
    uint8_t  amplitude;     // 0-128
    bool     sweep_active;
    uint32_t sweep_target;
    int32_t  sweep_delta;
    bool     sweep_up;
} poly_voice_t;

static poly_voice_t s_poly_voices[POLY_NUM_VOICES];
```

### ISR (16kHz, 62.5µs budget)

Each tick:
1. For each voice: advance phase, apply sweep if active, look up sine LUT, scale by amplitude
2. Sum all voices, divide by active count, clamp to 0-255
3. Update LEDC duty

Estimated ~3-4µs per voice × 12 = ~36-48µs + ~10µs base = ~46-58µs. Within 62.5µs budget.

Sine-only waveform for poly mode (no triangle/square — not needed for THX).

### Timer reconfiguration

- Poly start: reconfigure GPTimer alarm to 1MHz/16kHz = 62 ticks
- Poly stop: restore to 1MHz/32kHz = 31 ticks if mono resumes
- LEDC PWM frequency stays at 156kHz (8-bit) — still well above audible

### Mutual exclusion

Poly mode is mutually exclusive with mono DDS and FIFO modes. Starting poly stops any running mono/FIFO. Starting mono/FIFO stops poly.

No fullwave/polarity support in poly mode (not needed, keeps ISR simple).

## Python C Bindings

```python
pz_drive.pwm_poly_start()                           # init 16kHz timer, start ISR
pz_drive.pwm_poly_stop()                             # stop ISR, standby
pz_drive.pwm_poly_is_running()                       # bool
pz_drive.pwm_poly_set_voice(index, freq_hz, amplitude)  # immediate
pz_drive.pwm_poly_sweep_voice(index, target_hz, duration_ms)  # ISR-driven sweep
```

## Python PzActuator integration

No PzActuator wrapper needed — THX function calls `pz_drive` directly since it's a special-purpose mode. DRV2665 init/standby handled in the Python THX function.

## THX Deep Note Sequence (music.py)

```python
def thx(gain=100, duration_ms=10000):
    # Target: D chord across octaves + A for the fifth
    targets = [73, 73, 146, 146, 293, 293, 293, 587, 587, 587, 440, 440]

    # Phase 1: random cluster 200-400Hz, hold 2s
    # Phase 2: sweep all voices to targets over duration_ms
    # Phase 3: sustain 3s
    # Phase 4: stop
```

## Timing budget analysis

| Component | Time per tick |
|-----------|--------------|
| Phase accumulation (12 voices) | ~12µs |
| Sine LUT lookup (12 voices) | ~12µs |
| Amplitude scaling + sum | ~12µs |
| Division + clamp | ~2µs |
| LEDC update | ~3µs |
| **Total** | **~41µs** |
| **Budget (16kHz)** | **62.5µs** |
| **Margin** | **~21µs** |

## Core Pinning

The GPTimer ISR is pinned to **core 1** to avoid starving USB-CDC (TinyUSB runs on core 0). In ESP-IDF v5.5.1, `gptimer_register_event_callbacks()` lazily calls `esp_intr_alloc_intrstatus()`, which allocates the interrupt on the calling core. Since MicroPython runs on core 0, we spawn a temporary FreeRTOS task pinned to core 1 that registers the ISR callback, then self-deletes.

This applies to **all** PWM modes (mono DDS, sample playback, and poly), since they share the same GPTimer and ISR. The FIFO background task (fifo.c) is similarly pinned to core 1 via `xTaskCreatePinnedToCore`.
