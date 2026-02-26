# Analog Sine Wave Generation via PWM

**Date:** 2026-02-25
**Status:** Design approved

## Problem

The current digital path (I2C FIFO at 8 kHz) has a 6.83x frequency scaling issue and cannot easily fan out to multiple DRV2665/DRV2700 boards (same I2C address 0x59). An analog signal path would:

- Eliminate the frequency scaling problem
- Allow passive fan-out to 6+ actuator boards
- Support future DRV2700 (analog-only, same footprint as DRV2665)
- Simplify firmware (no FIFO management for the analog path)

## Hardware Design

### Signal Path

```
ESP32-S3 GPIO5 (LEDC PWM)
    │
    ├─ 312.5 kHz @ 8-bit (or 78.1 kHz @ 10-bit)
    │
    R13 (390 Ω)
    │
    ├── C9 (0.1 µF) ── GND
    │
    JP7 (bridge pads 1-2)
    │
    PWM_FILTERED ── IN+ (DRV2665 analog input)
                    IN- = GND
```

### Filter Characteristics

- **Topology:** Single-pole passive RC low-pass
- **Components:** R13 = 390 Ω, C9 = 0.1 µF (existing footprints, 0603)
- **Cutoff frequency:** fc = 1/(2π × 390 × 100nF) = 4.08 kHz
- **At 400 Hz signal:** -0.04 dB attenuation (negligible)
- **At 312.5 kHz carrier (8-bit):** -37.7 dB attenuation (~13 mV ripple on 1V signal)
- **At 78.1 kHz carrier (10-bit):** -25.6 dB attenuation (~52 mV ripple on 1V signal)
- **Additional filtering:** 10 nF piezo actuators act as further LPF

### PCB Changes

**Populate:**
- R13 (390 Ω, 0603)
- C9 (0.1 µF, 0603)
- JP7: bridge pads 1-2 (passive path)

**DNP (do not populate):**
- U4 (LM324), R6-R12, C5-C8 (active filter stages not needed)

### DRV2665 Configuration

- Switch to analog input mode: CTRL1 bit 2 = 1 (`DRV2665_INPUT_ANALOG`)
- IN- connected to GND (single-ended)
- Signal at IN+ swings 0-3.3V; sine wave centered at ~1.65V (50% duty)

## Software Design

### Compile-Time Configuration

```c
#define PWM_GPIO            5
#define PWM_DEFAULT_RESOLUTION  8       // 8 or 10 bits
#define PWM_SAMPLE_RATE_HZ     32000   // DDS update rate
```

Derived values:
- 8-bit: 80 MHz / 256 = 312.5 kHz carrier, 256 duty levels
- 10-bit: 80 MHz / 1024 = 78.125 kHz carrier, 1024 duty levels

### DDS (Direct Digital Synthesis)

Fixed-rate GPTimer interrupt at `PWM_SAMPLE_RATE_HZ` (32 kHz). Phase accumulator approach:

```c
// Setup
uint32_t phase_accumulator = 0;
uint32_t phase_step = (uint32_t)((frequency / PWM_SAMPLE_RATE_HZ) * (1ULL << 32));

// Each timer interrupt (32 kHz):
phase_accumulator += phase_step;
uint8_t index = phase_accumulator >> (32 - LUT_BITS);
ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, sine_lut[index]);
ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
```

- 50 Hz: 640 samples/cycle
- 400 Hz: 80 samples/cycle
- Sub-Hz frequency resolution
- Frequency changes are instantaneous (update `phase_step`)

### Sine Lookup Table

Shared between analog and digital paths. Generated in C at compile time or init:

```c
// 256-entry table, values 0-255 (8-bit) or 0-1023 (10-bit)
// Scaled to PWM resolution at runtime
static const uint8_t sine_lut_8bit[256];   // half-wave symmetry optimization optional
```

### MicroPython API

```python
import pz_actuator

pz_actuator.init()

# Analog path (PWM + filter)
pz_actuator.set_frequency_analog(250)      # 250 Hz sine wave
pz_actuator.set_frequency_analog(0)        # DC fully on

# Digital path (I2C FIFO) — sine generation moved from Python to C
pz_actuator.set_frequency_digital(250)     # 250 Hz sine wave

# Runtime resolution switch
pz_actuator.set_pwm_resolution(10)         # switch to 10-bit / 78 kHz

# Common
pz_actuator.stop()                         # stop whichever is active
pz_actuator.set_gain(100)                  # gain setting (applies to both)
```

### Digital Path Refactor

The existing `set_waveform(buf)` + Python sine generation moves into C:

- `set_frequency_digital(hz)` generates a sine LUT internally and starts the FreeRTOS FIFO-feeding task
- `set_waveform(buf)` remains as an advanced API for custom waveforms
- Both paths share the same sine table code

### Modes

| Mode | PWM | DRV2665 CTRL1 | Timer | FIFO Task |
|------|-----|---------------|-------|-----------|
| Analog sine | running, DDS updates | INPUT_ANALOG | active @ 32 kHz | stopped |
| Analog DC | fixed 100% duty | INPUT_ANALOG | stopped | stopped |
| Digital sine | stopped | INPUT_DIGITAL | stopped | active |
| Off | stopped / 0% | either | stopped | stopped |

## Hardware Pins (updated)

| Function        | GPIO |
|-----------------|------|
| I2C SDA         | 47   |
| I2C SCL         | 21   |
| SPI MOSI        | 6    |
| SPI MISO        | 7    |
| SPI SCK         | 9    |
| SPI CS          | 10   |
| Polarity toggle | 34   |
| **PWM out**     | **5** |
