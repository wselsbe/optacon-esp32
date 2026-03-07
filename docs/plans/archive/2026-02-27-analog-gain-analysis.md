# Analog Gain Analysis & Amplitude Scaling

**Date:** 2026-02-27

## Signal Chain

```
ESP32 GPIO5 (3.3V) → R13 (390Ω) → C9 (100nF) → [CH4 probe] → C3 (0.1µF AC coupling) → DRV2665 IN+
                                                                                          DRV2665 IN- → C_IN → GND
DRV2665 VDD = 5V (from USB)
DRV2665 VBST ≈ 100V (set by R1/R2 feedback divider)
DRV2665 OUT+ → actuators (single-ended, NOT differential)
DRV2665 OUT- → not connected to actuators
```

## Key Findings

### 1. IN+/IN- is a differential input

The DRV2665 amplifier measures V_diff = V(IN+) - V(IN-). Both pins have internal bias (~2.5V, roughly VDD/2). Since IN- is AC-coupled to GND, it sits at the internal bias with no AC signal. The full AC swing from the filtered PWM on IN+ becomes the differential input.

### 2. Maximum recommended differential input is 1.8V pk-pk

From the DRV2665 datasheet (Recommended Operating Conditions): **VIN = 1.8V differential max**. This is the full-scale input that produces the rated output at each gain setting.

### 3. We were driving 3.3V pk-pk — nearly 2× over

The sine LUT swings 0–255, producing 0–100% PWM duty → 0 to 3.3V after RC filtering → 3.3V pk-pk AC after C3 coupling. This is 1.83× the max recommended input.

### 4. Output is single-ended (OUT+ only)

The DRV2665 has a fully-differential amplifier with OUT+ and OUT- swinging in opposite phase around VBST/2 (~50V). The datasheet's VPP specs (50/100/150/200 VPP) refer to the **differential** output (OUT+ minus OUT-).

**However, our board only uses OUT+ to drive the actuators (single-ended).** This means:
- OUT+ swings between ~0V and ~VBST (~100V), centered at VBST/2
- **The actuator sees half the differential spec** — max ~VBST pk-pk ≈ 100V
- CH1 (VPP) directly measures the actuator drive voltage

### 5. VBST ≈ 100V (set by feedback resistors)

The boost converter output is set by the R1/R2 feedback divider to approximately 100V (datasheet max 105V). This is the hard ceiling for OUT+ single-ended swing.

### 6. Gain measurements (single-ended, actual actuator voltage)

| Gain | dB (analog) | Voltage gain | CH1 = actuator V | Clipping? |
|------|-------------|-------------|-----------------|-----------|
| 25V  | 28.8 | 27.5× | 46.4V pk-pk | No — headroom left |
| 50V  | 34.8 | 55.0× | 92.0V pk-pk | No — close to VBST |
| 75V  | 38.4 | 83.2× | 104.8V pk-pk | Yes — at VBST |
| 100V | 40.7 | 108.4× | 104.8V pk-pk | Yes — hard clip at VBST |

The gain math works out for single-ended: the DRV2665 amplifier gain applies to the differential output, but OUT+ only carries half. So: single-ended = input × gain / 2.

- Gain 25: 3.3V × 27.5 / 2 = 45.4V → measured 46.4V ✓
- Gain 50: 3.3V × 55.0 / 2 = 90.75V → measured 92.0V ✓

### 7. Optimal single-ended gain strategy

Since we're limited to ~VBST ≈ 100V single-ended, the useful gain range is:
- **Gain=50 with full PWM (3.3V)**: 92V — near-optimal, 92% of VBST
- **Gain=100 with amplitude=48% (1.8V)**: 89.6V — also good, clean sine
- **Gain=25 with full PWM**: 46V — only uses 46% of available range

For maximum actuator drive, **gain=50 at full amplitude** or **gain=100 at ~48% amplitude** are equivalent. Higher gain with lower amplitude gives finer software control.

### 8. With proper 1.8V input, gain settings match datasheet (differential)

These are the datasheet's differential specs. For our single-ended setup, divide by 2:

| Gain | 1.8V input → differential | Single-ended (OUT+) | vs VBST |
|------|--------------------------|--------------------|---------|
| 25V  | 49.5V | ~25V | 25% of VBST |
| 50V  | 99.0V | ~50V | 50% of VBST |
| 75V  | 149.8V | ~75V | 75% of VBST |
| 100V | 195.1V | ~98V | ~98% of VBST |

So at gain=100 with 1.8V input, OUT+ swings nearly the full VBST range — this is the sweet spot for single-ended use.

## Solution: PWM Amplitude Scaling

### Implementation

Added `amplitude` kwarg to `start()` (0–100%, default 100):

```python
pz_actuator.start(gain=100, amplitude=48)   # 1.8V input → ~90V clean single-ended
pz_actuator.start(gain=50, amplitude=100)    # 3.3V input → ~92V (slight overdrive but works)
pz_actuator.start(gain=25, amplitude=100)    # 3.3V input → ~46V
pz_actuator.start(gain=100, amplitude=25)    # 0.8V input → ~45V (fine volume control)
```

Internally, the ISR scales the sine LUT around its midpoint:
```c
int32_t raw = (int32_t)sine_lut_8bit[index] - 128;   // -128 to +127
int32_t scaled = 128 + ((raw * s_amplitude) >> 7);     // back to 0-255
```

Where `s_amplitude` is 0–128 (mapped from 0–100%). `amplitude=48` → internal 61 → ~1.8V pk-pk.

### Amplitude-to-output table (single-ended actuator voltage)

| amplitude (%) | Input pk-pk | Gain=25 | Gain=50 | Gain=75 | Gain=100 |
|--------------|-------------|---------|---------|---------|----------|
| 100 | 3.3V | 46V | 92V | ~100V¹ | ~100V¹ |
| 48 | 1.6V | 22V | 44V | 66V | 87V |
| 55 | 1.8V | 25V | 50V | 75V | 98V |
| 27 | 0.9V | 12V | 25V | 37V | 49V |
| 14 | 0.45V | 6V | 12V | 19V | 24V |

¹ Clipped at VBST ≈ 100V

## Note: PWM Scaling vs Supply Voltages

The correct `amplitude` percentage to achieve the DRV2665 full-scale input (1.8V pk-pk differential) depends on the ratio of two voltages:

- **ESP32 GPIO voltage** (VDD_3.3): determines the PWM output swing (0 to VDD_3.3)
- **DRV2665 full-scale input**: 1.8V pk-pk (datasheet spec, independent of DRV2665 VDD)

The ideal amplitude = **1.8V / VDD_ESP32 × 100%**. With a nominal 3.3V regulator, this is 54.5%.

However, the ESP32's 3.3V regulator output depends on the USB supply voltage, temperature, and load. If the USB supply is 5.0V the regulator may output 3.30V; at 4.8V it might output 3.20V. This changes the PWM full-scale and thus the ideal amplitude percentage.

### Options to make this robust:

1. **Compile-time constant** (simplest): Define `PWM_VDD_MV = 3300` and compute the default amplitude automatically: `amplitude = 1800 * 128 / PWM_VDD_MV`. This is good enough if the regulator is stable.

2. **Runtime ADC measurement** (future motherboard revision): Add a voltage divider from the ESP32's 3.3V rail (or even the 5V USB rail) to an ADC pin. At startup or periodically, measure the actual supply voltage and auto-adjust the amplitude. This gives ±1% accuracy regardless of supply variation.

3. **Measure actuator board supply** (future, most accurate): If the actuator board has its own regulator (or if the 5V USB path to DRV2665 has voltage drop), measuring the voltage at the actuator board connector with an ADC-connected divider would let the ESP32 compensate for cable and connector losses too.

### Example ADC-based auto-calibration (future):

```python
# On the ESP32, with a voltage divider (e.g., 10k/10k) from 3.3V rail to ADC pin:
import machine
adc = machine.ADC(machine.Pin(XX))
vdd_mv = adc.read_uv() * 2 / 1000  # account for divider ratio
ideal_amplitude = int(1800 / vdd_mv * 100)
pz_actuator.start(gain=100, amplitude=ideal_amplitude)
```

## IN- Design Considerations (for future revision)

### Current: IN- AC-coupled to GND
Works fine. The internal bias sets the DC operating point. Only IN+ carries signal.

### Option: IN- driven by inverted GPIO
Provides common-mode noise rejection (boost converter switching noise cancels). Does NOT increase max differential swing — still limited to 3.3V pk-pk per GPIO. Uses an extra GPIO + RC filter + coupling cap. Worth considering if noise is a problem.

### Option: IN- to voltage divider
No benefit. AC coupling cap blocks DC, so the divider voltage never reaches the pin — IN- sits at internal bias regardless.

### Option: IN- AC-coupled to GPIO held LOW
Effectively the same as AC-coupled to GND. The AC coupling cap blocks the DC level, and a LOW GPIO (~0V) behaves the same as a GND connection. No benefit over the current design.

## Single-Ended vs Differential Actuator Connection

The current board connects only OUT+ to the actuators, giving a maximum swing of ~VBST ≈ 100V pk-pk. If a future revision connects actuators differentially (between OUT+ and OUT-), the full 200 VPP output becomes available, doubling the drive voltage. This would require routing OUT- to the actuator board and changes to the shift register/mux design.
