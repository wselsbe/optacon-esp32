# Analog Gain Analysis & Amplitude Scaling

**Date:** 2026-02-27

## Signal Chain

```
ESP32 GPIO5 (3.3V) → R13 (390Ω) → C9 (100nF) → [CH4 probe] → C3 (0.1µF AC coupling) → DRV2665 IN+
                                                                                          DRV2665 IN- → C_IN → GND
DRV2665 VDD = 5V (from USB)
DRV2665 OUT+/OUT- → piezo actuator (differential)
```

## Key Findings

### 1. IN+/IN- is a differential input

The DRV2665 amplifier measures V_diff = V(IN+) - V(IN-). Both pins have internal bias (~2.5V, roughly VDD/2). Since IN- is AC-coupled to GND, it sits at the internal bias with no AC signal. The full AC swing from the filtered PWM on IN+ becomes the differential input.

### 2. Maximum recommended differential input is 1.8V pk-pk

From the DRV2665 datasheet (Recommended Operating Conditions): **VIN = 1.8V differential max**. This is the full-scale input that produces the rated output at each gain setting.

### 3. We were driving 3.3V pk-pk — nearly 2× over

The sine LUT swings 0–255, producing 0–100% PWM duty → 0 to 3.3V after RC filtering → 3.3V pk-pk AC after C3 coupling. This is 1.83× the max recommended input.

### 4. CH1 measures single-ended (half the differential output)

The DRV2665 output is fully differential. OUT+ and OUT- swing in opposite phase around VBST/2 (~52.5V). The actual actuator drive voltage is **2× the CH1 (single-ended) measurement**.

### 5. Gain measurements explained

| Gain | dB (analog) | Voltage gain | CH1 measured | Actual differential | Expected (1.8V input) |
|------|-------------|-------------|--------------|--------------------|-----------------------|
| 25V  | 28.8 | 27.5× | 46.4V pk-pk | **92.8V pk-pk** | 50 VPP |
| 50V  | 34.8 | 55.0× | 92.0V pk-pk | **184V pk-pk** | 100 VPP |
| 75V  | 38.4 | 83.2× | 104.8V pk-pk | **~210V (clipped)** | 150 VPP |
| 100V | 40.7 | 108.4× | 104.8V pk-pk | **~210V (clipped)** | 200 VPP |

Verification: At gain=25, 3.3V × 27.5 = 90.75V differential ÷ 2 = 45.4V single-ended. Measured 46.4V. ✓

### 6. With proper 1.8V input, gain settings match datasheet

- Gain 25: 1.8 × 27.5 = **49.5V** → 50 VPP ✓
- Gain 50: 1.8 × 55.0 = **99.0V** → 100 VPP ✓
- Gain 75: 1.8 × 83.2 = **149.8V** → 150 VPP ✓
- Gain 100: 1.8 × 108.4 = **195.1V** → 200 VPP ✓

## Solution: PWM Amplitude Scaling

### Implementation

Added `amplitude` kwarg to `start()` (0–100%, default 100):

```python
pz_actuator.start(gain=100, amplitude=55)  # 1.8V input → 200 VPP clean
pz_actuator.start(gain=25, amplitude=55)   # 1.8V input → 50 VPP clean
pz_actuator.start(gain=100, amplitude=100) # 3.3V input → clips at ~210V
```

Internally, the ISR scales the sine LUT around its midpoint:
```c
int32_t raw = (int32_t)sine_lut_8bit[index] - 128;   // -128 to +127
int32_t scaled = 128 + ((raw * s_amplitude) >> 7);     // back to 0-255
```

Where `s_amplitude` is 0–128 (mapped from 0–100%). `amplitude=55` → internal 70 → 1.8V pk-pk.

### Amplitude-to-output table (at each gain, no clipping)

| amplitude (%) | Input pk-pk | Gain=25 diff | Gain=50 diff | Gain=75 diff | Gain=100 diff |
|--------------|-------------|-------------|-------------|-------------|--------------|
| 100 | 3.3V | 91V | 182V¹ | 210V¹ | 210V¹ |
| 55 | 1.8V | 50V | 99V | 150V | 195V |
| 27 | 0.9V | 25V | 50V | 75V | 98V |
| 14 | 0.45V | 12V | 25V | 37V | 49V |
| 1 | 0.03V | 0.9V | 1.7V | 2.5V | 3.4V |

¹ Clipped at 2×VBST ≈ 210V

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
