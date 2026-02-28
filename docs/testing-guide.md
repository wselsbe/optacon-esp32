# Hardware Testing Guide

This guide covers functional testing of the Optacon motherboard and actuator board using an oscilloscope, power supply, and optionally a logic analyzer.

## Equipment

- **Oscilloscope** -- 4-channel recommended (e.g., Siglent SDS series). At least 2 channels required.
- **Power supply** -- adjustable voltage and current limit (e.g., Siglent SPD series). Set to 5V, 0.5-0.6A limit.
- **Logic analyzer** -- optional, for debugging SPI/I2C bus traffic (e.g., Saleae or fx2lafw-compatible).
- **USB cable** -- USB-C to connect the ESP32-S3 for REPL access.
- **Oscilloscope probes** -- 4x passive probes, all set to 1x attenuation unless noted.

## Oscilloscope Setup

### General Configuration

**Probe attenuation**: Set all probes to **1x** in both the probe hardware switch and the oscilloscope channel settings. The signals in this system are all low-voltage (0-5V logic, 0-3.3V PWM) except for the DRV2665 output which can reach up to ~100V. For the DRV2665 output, use a **10x** probe or reduce the channel scale accordingly.

**Channel colors** (Siglent SDS default): CH1 = yellow, CH2 = magenta/pink, CH3 = cyan/blue, CH4 = green. This guide references these colors.

**Vertical scale**: Start with 1V/div for logic signals (SPI CS, polarity GPIO), 500mV/div for PWM, and 20V/div for DRV2665 output. Adjust as needed.

**Vertical offset**: When viewing multiple channels simultaneously, offset each trace so they don't overlap. A good starting point:
- CH1: +3 div offset
- CH2: +1 div offset
- CH3: -1 div offset
- CH4: -3 div offset

**Timebase**: For 250 Hz signals, start at 2 ms/div (one full period visible). For verifying latch timing, zoom to 500 us/div or less. For high-frequency digital FIFO signals (1-4 kHz), use 200 us/div.

**Triggering**: Trigger on the channel with the cleanest edge. For analog waveforms, trigger on the DRV2665 output (rising edge, auto mode). For latch timing, trigger on the polarity pin or SPI CS (rising edge, normal mode). Set trigger level to ~1.5V for 3.3V logic signals.

**Measurements**: Use the oscilloscope's built-in measurement function for:
- **Frequency** -- verify output frequency matches configuration
- **Vpp** -- verify output amplitude
- **Rise/Fall time** -- check DRV2665 output transitions during polarity toggle

### Probe Compensation

Before first use, connect each probe to the oscilloscope's calibration output (square wave, typically 1 kHz) and adjust the trimmer on the probe until the square wave edges are flat (no overshoot or rounding). This only needs to be done once per probe.

## Power Supply Setup

1. Set voltage to **5.0V**
2. Set current limit to **0.5A** (increase to 0.6A if board trips during fullwave square wave tests)
3. Enable output before connecting USB (the board draws power from both USB and external supply)
4. Monitor current during tests -- typical idle current is ~50 mA, active output is 100-300 mA depending on mode and number of active actuator channels

**Current limit warnings**: The board may draw excess current during certain transitions, particularly when switching all shift register pins off simultaneously during fullwave mode. If the supply enters CC (constant current) mode, stop the output from the REPL (`pa.stop()`) and investigate.

## REPL Access

Connect to the board via USB:

```bash
mpremote connect COM7
```

Import the actuator controller:

```python
from pz_actuator_py import PzActuator
pa = PzActuator()
```

To enter bootloader mode for reflashing:

```python
import board_utils
board_utils.enter_bootloader()
```

---

## Test Groups

Tests are organized by probe placement to minimize physical reconnection. Each group specifies where to connect each oscilloscope probe. Complete all tests in a group before moving probes.

---

## Group 1: Analog Path Verification

Verify that the analog signal path works: PWM output, RC filter, DRV2665 amplification, and actuator output.

### Probe Placement

| Channel | Signal    | Test Point                        | Attenuation | Scale   |
|---------|-----------|-----------------------------------|-------------|---------|
| CH1     | PWM       | GPIO 5 (LEDC output)             | 1x          | 1V/div  |
| CH2     | IN+       | DRV2665 analog input (after RC)  | 1x          | 500mV/div |
| CH3     | Actuator  | one actuator output pin (e.g. pin 4) | 10x     | 20V/div |
| CH4     | OUT+      | DRV2665 output (before HV509)    | 10x         | 20V/div |

Trigger: CH2 (IN+), rising edge, auto mode.

Timebase: 2 ms/div for 250 Hz.

### Test 1.1: Basic Analog Sine (250 Hz)

```python
pa.set_frequency_analog(250)
pa.set_all(1)
pa.start()
```

**Verify:**
- CH1 (PWM): fast-switching PWM with varying duty cycle
- CH2 (IN+): smooth sine wave centered around ~1.3V DC bias (AC-coupled through C3)
- CH4 (OUT+): amplified sine wave, ~50 Vpp at gain=100
- CH3 (Actuator): same as OUT+ when pin is enabled
- Frequency measurement on CH2 or CH4 reads ~250 Hz

```python
pa.stop()
```

### Test 1.2: Frequency Sweep

Test several frequencies to verify the DDS frequency accuracy:

```python
import time
for freq in [50, 100, 200, 250, 300, 400]:
    pa.set_frequency_analog(freq)
    pa.set_all(1)
    pa.start()
    # Verify frequency on scope, check waveform shape
    time.sleep(3)
    pa.stop()
    time.sleep(0.5)
```

**Verify:** Each frequency is within ~1% of target. Waveform shape remains sinusoidal across the range.

### Test 1.3: Waveform Types

```python
pa.set_frequency_analog(250, waveform='sine')
pa.set_all(1)
pa.start()
# Verify: smooth sine on CH2/CH4
pa.stop()

pa.set_frequency_analog(250, waveform='triangle')
pa.set_all(1)
pa.start()
# Verify: triangle wave on CH2, may be slightly rounded by RC filter
pa.stop()

pa.set_frequency_analog(250, waveform='square')
pa.set_all(1)
pa.start()
# Verify: square wave on CH1 (PWM stays at fixed duty), rounded square on CH2
pa.stop()
```

### Test 1.4: Amplitude Control

```python
pa.set_frequency_analog(250, amplitude=50)
pa.set_all(1)
pa.start()
# Verify: CH4 output is roughly half the amplitude of amplitude=100
pa.stop()

pa.set_frequency_analog(250, amplitude=100)
pa.set_all(1)
pa.start(gain=50)
# Verify: CH4 output is ~25 Vpp (half of gain=100)
pa.stop()
```

### Test 1.5: DC Mode

```python
pa.set_frequency_analog(0)
pa.set_all(1)
pa.start()
# Verify: CH1 (PWM) at constant high duty, CH4 (OUT+) at steady DC level
pa.stop()
```

---

## Group 2: Fullwave Analog Verification

Verify fullwave rectification and polarity toggling. This group adds the polarity signal.

### Probe Placement

| Channel | Signal    | Test Point                        | Attenuation | Scale   |
|---------|-----------|-----------------------------------|-------------|---------|
| CH1     | Polarity  | GPIO 12 or 13 (POL_A/B)          | 1x          | 1V/div  |
| CH2     | IN+       | DRV2665 analog input (after RC)  | 1x          | 500mV/div |
| CH3     | SPI CS/LE | GPIO 10 (latch enable)           | 1x          | 2V/div  |
| CH4     | OUT+      | DRV2665 output                    | 10x         | 20V/div |

Trigger: CH1 (Polarity), rising edge, normal mode.

Timebase: 2 ms/div for 250 Hz.

### Test 2.1: Fullwave Sine (250 Hz)

```python
pa.set_frequency_analog(250, fullwave=True)
pa.set_all(1)
pa.start()
```

**Verify:**
- CH1 (Polarity): toggling at 250 Hz (two toggles per period = one full cycle)
- CH2 (IN+): rectified sine (only positive half-cycles, twice the frequency)
- CH4 (OUT+): polarity reversal visible -- waveform alternates direction each half-cycle
- The polarity edges on CH1 align with the zero-crossings of the waveform on CH2

```python
pa.stop()
```

### Test 2.2: Fullwave with Dead Time

```python
pa.set_frequency_analog(250, fullwave=True, dead_time=3)
pa.set_all(1)
pa.start()
```

**Verify:**
- CH2 (IN+): brief zero-output period visible near each zero-crossing
- CH4 (OUT+): smoother polarity transitions compared to dead_time=0

```python
pa.stop()
```

### Test 2.3: Fullwave with Phase Advance

```python
pa.set_frequency_analog(250, fullwave=True, phase_advance=3)
pa.set_all(1)
pa.start()
```

**Verify:**
- CH1 (Polarity): toggle edge occurs slightly *before* the waveform zero-crossing on CH2
- This compensates for the DRV2665 output lag

```python
pa.stop()
```

### Test 2.4: Shift Register Latch Timing (Fullwave)

While fullwave output is running, toggle a pin and verify the latch happens at the right moment:

```python
pa.set_frequency_analog(250, fullwave=True)
pa.set_all(1)
pa.start()

# Now toggle a pin -- the actual latch should be synchronized
pa.set_pin(4, 0)
# Watch CH3 (SPI CS/LE): the rising edge (latch) should coincide with
# a polarity toggle on CH1, not the moment set_pin was called
pa.set_pin(4, 1)
```

**Verify:** CH3 (LE) rising edge aligns with CH1 (polarity) edge, confirming ISR-synchronized latching.

```python
pa.stop()
```

### Test 2.5: Shift Register Latch Timing (Non-Fullwave)

```python
pa.set_frequency_analog(250, fullwave=False)
pa.set_all(1)
pa.start()

pa.set_pin(4, 0)
# CH3 (LE) rising edge should align with cycle start (phase wrap),
# which corresponds to the trough of the sine wave
pa.set_pin(4, 1)
```

```python
pa.stop()
```

---

## Group 3: Digital FIFO Verification

Verify the digital signal path (I2C FIFO to DRV2665 internal DAC).

### Probe Placement

| Channel | Signal    | Test Point                        | Attenuation | Scale   |
|---------|-----------|-----------------------------------|-------------|---------|
| CH1     | Polarity  | GPIO 12 or 13 (POL_A/B)          | 1x          | 1V/div  |
| CH2     | Actuator  | one actuator output (e.g. pin 4) | 10x         | 20V/div |
| CH3     | SPI CS/LE | GPIO 10 (latch enable)           | 1x          | 2V/div  |
| CH4     | OUT+      | DRV2665 output                    | 10x         | 20V/div |

Trigger: CH4 (OUT+), rising edge, auto mode.

Timebase: 2 ms/div for 250 Hz, 500 us/div for 1 kHz.

### Test 3.1: Digital Sine (250 Hz)

```python
pa.set_frequency_digital(250)
pa.set_all(1)
pa.start()
```

**Verify:**
- CH4 (OUT+): sine wave with slight staircase stepping (8 kHz DAC rate = 32 samples per period)
- Frequency measurement reads ~247-250 Hz
- CH2 (Actuator): matches CH4 when pin enabled

```python
pa.stop()
```

### Test 3.2: Digital Frequency Sweep

```python
import time
for freq in [50, 100, 200, 250, 500, 1000]:
    pa.set_frequency_digital(freq)
    pa.set_all(1)
    pa.start()
    # Verify frequency and waveform on scope
    time.sleep(3)
    pa.stop()
    time.sleep(0.5)
```

**Verify:** Each frequency is within ~2% of target. Higher frequencies have fewer samples per period (more visible stepping).

### Test 3.3: Digital Fullwave

```python
pa.set_frequency_digital(250, fullwave=True)
pa.set_all(1)
pa.start()
```

**Verify:**
- CH1 (Polarity): toggling at 250 Hz
- CH4 (OUT+): rectified waveform with polarity reversal
- Polarity toggles at period boundaries (when FIFO write index wraps)

```python
pa.stop()
```

### Test 3.4: Digital Waveform Types

```python
pa.set_frequency_digital(250, waveform='triangle')
pa.set_all(1)
pa.start()
# Verify: triangle shape on CH4
pa.stop()

pa.set_frequency_digital(250, waveform='square')
pa.set_all(1)
pa.start()
# Verify: square wave on CH4
pa.stop()
```

### Test 3.5: Gain Settings

```python
pa.set_frequency_digital(250)
pa.set_all(1)

pa.start(gain=100)
# Measure Vpp on CH4
pa.stop()

pa.start(gain=50)
# Verify: Vpp is roughly half of gain=100
pa.stop()

pa.start(gain=25)
# Verify: Vpp is roughly quarter of gain=100
pa.stop()
```

---

## Group 4: Pin-Level Actuator Verification

Verify individual actuator channels respond correctly. This group requires moving CH2 between actuator pins.

### Probe Placement

| Channel | Signal    | Test Point                 | Attenuation | Scale   |
|---------|-----------|----------------------------|-------------|---------|
| CH1     | Polarity  | GPIO 12 or 13              | 1x          | 1V/div  |
| CH2     | Actuator  | actuator pin under test    | 10x         | 20V/div |
| CH3     | SPI CS/LE | GPIO 10                    | 1x          | 2V/div  |
| CH4     | OUT+      | DRV2665 output             | 10x         | 20V/div |

Trigger: CH4 (OUT+), rising edge, auto mode.

### Test 4.1: Individual Pin Toggle

```python
pa.set_frequency_analog(250)
pa.set_all(0)
pa.start()

# Probe CH2 on the actuator pin under test (e.g., pin 4)
pa.set_pin(4, 1)
# Verify: CH2 shows waveform matching CH4
# Verify: CH3 (LE) shows latch pulse

pa.set_pin(4, 0)
# Verify: CH2 goes flat (no signal)

pa.stop()
```

### Test 4.2: Pin Scanning

Cycle through all 20 pins. Move CH2 probe to verify a few representative pins:

```python
import time
pa.set_frequency_analog(250)
pa.start()

for pin in range(20):
    pa.set_all(0)
    pa.set_pin(pin, 1)
    print(f"Pin {pin} active")
    time.sleep(1)

pa.stop()
```

### Test 4.3: All Pins Simultaneously

```python
pa.set_frequency_analog(250)
pa.set_all(1)
pa.start()
# Verify: CH2 (any pin) and CH4 (OUT+) match
# Monitor power supply current -- should be higher with all 20 channels active
pa.stop()
```

---

## Group 5: I2C and SPI Bus Debugging (Logic Analyzer)

Optional. Use a logic analyzer to inspect raw bus traffic when debugging communication issues.

### Logic Analyzer Connections

| Channel | Signal | Test Point |
|---------|--------|------------|
| A0      | SDA    | GPIO 47    |
| A1      | SCL    | GPIO 21    |
| A2      | MOSI   | GPIO 6     |
| A3      | MISO   | GPIO 7     |
| A4      | SCLK   | GPIO 9     |
| A5      | SS/LE  | GPIO 10    |
| A6      | POL    | GPIO 12    |

### Test 5.1: I2C Register Access

Capture while reading a DRV2665 register:

```python
import pz_drive
pz_drive.i2c_read(0x00)  # read STATUS register
```

**Verify:** Logic analyzer shows I2C transaction: START, write 0x59+W, register 0x00, RESTART, read 0x59+R, data byte, STOP.

### Test 5.2: SPI Shift Register Write

```python
import pz_drive
pz_drive.sr_write(0x02000000)  # set pin 0 only
```

**Verify:** 32-bit SPI transaction on MOSI, followed by LE rising edge on SS/LE.

### Test 5.3: FIFO Streaming

Capture during digital playback to verify I2C FIFO writes:

```python
pa.set_frequency_digital(250)
pa.set_all(1)
pa.start()
# Capture ~20ms of I2C traffic
pa.stop()
```

**Verify:** Repeated bulk I2C writes to register 0x0B, approximately every 8 ms, with ~64 bytes per write.

---

## Troubleshooting

**No output on scope:**
- Check that `pa.start()` was called after `set_frequency_*`
- Verify at least one actuator pin is enabled (`pa.set_all(1)`)
- Check power supply is enabled and not in CC mode
- Try `pz_drive.i2c_read(0x00)` -- should return 0 or 2 (FIFO_EMPTY), not -1

**Output frequency is wrong:**
- Analog: verify `hz` parameter is 50-400
- Digital: if frequency is ~6-7x too slow, the FIFO fill timing is broken (vTaskDelay vs esp_timer)

**Board resets during test:**
- Check power supply current limit (increase to 0.6A)
- Avoid turning all shift register pins off simultaneously during fullwave mode
- Check for I2C bus lockup: `pz_drive.i2c_read(0x00)` should respond

**No polarity toggle in fullwave mode:**
- Verify `fullwave=True` was passed to `set_frequency_*`
- Check probe on GPIO 12 or 13 -- should see toggling square wave at the configured frequency

**Latch not synchronized:**
- The LE rising edge on GPIO 10 should align with waveform events
- In fullwave: aligns with polarity toggle (zero-crossing)
- Without fullwave: aligns with cycle start (phase accumulator wrap)
- If LE pulses immediately on `set_pin()`, the ISR/task may not be running

**Board won't enter bootloader:**
- Use `board_utils.enter_bootloader()` from the REPL (not mpremote's built-in reset)
- The COM port number changes in bootloader mode (e.g., COM5 -> COM4)
- Bootloader USB device shows VID:PID 303A:1001
