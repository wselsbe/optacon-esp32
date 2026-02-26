# API Refactor Overview & Hardware Test Plan

**Date:** 2026-02-26
**Branch:** main (commit 57401ff)

## API Changes Summary

### New API (configure-then-start pattern)

```python
import pz_actuator

pz_actuator.init()                          # I2C + SPI + PWM hardware init

# Configure mode (does NOT start output)
pz_actuator.set_frequency_analog(250)               # analog sine 250Hz, 8-bit PWM
pz_actuator.set_frequency_analog(250, resolution=10) # analog sine 250Hz, 10-bit PWM
pz_actuator.set_frequency_analog(0)                  # analog DC (100% duty)
pz_actuator.set_frequency_digital(250)               # digital sine 250Hz via I2C FIFO

# Start/stop
pz_actuator.start()                         # start with default 100Vpp gain
pz_actuator.start(gain=50)                  # start with 50Vpp gain
pz_actuator.stop()                          # stop output, DRV2665 to standby
pz_actuator.start()                         # restart same mode (no reconfigure needed)

# Pin control (independent of mode)
pz_actuator.set_all(True)                   # enable all 20 actuator channels
pz_actuator.set_pin(4, True)                # enable single channel
pz_actuator.is_running()                    # True if analog PWM or digital FIFO active
```

### Removed functions
- `set_waveform(buf)` -- replaced by `set_frequency_digital(hz)`
- `set_pwm_resolution(bits)` -- now `resolution=` kwarg on `set_frequency_analog()`
- `set_gain(gain)` -- now `gain=` kwarg on `start()`

### Internal mode state machine

```
         set_frequency_analog()          start()
NONE ──────────────────────────► ANALOG ────────► running (PWM + DRV2665 analog)
  │                                                    │
  │    set_frequency_digital()          start()        │ stop()
  └────────────────────────────► DIGITAL ───────► running (FIFO task + DRV2665 digital)
                                                       │
                                                  stop()  (mode preserved, can restart)
```

## Hardware Preparation

### Soldering the RC filter

Populate three components on the motherboard:

| Ref | Value | Package | Location on schematic |
|-----|-------|---------|----------------------|
| **R13** | 390 ohm | 0603 | "low pass filter" sheet, input series resistor |
| **C9** | 100 nF (0.1 uF) | 0603 | "low pass filter" sheet, shunt cap to GND |
| **JP7** | solder bridge pads 1-2 | SolderJumper_3 | selects passive path |

**JP7 orientation:** Looking at the 3-pad jumper, bridge the side labeled 1-2 (passive path). Do NOT bridge 2-3 (that's the active op-amp path which has no components populated).

**Do NOT populate:** U4 (LM324), R6-R12, C5-C8 (active filter, not needed).

### Scope probe placement

| Probe | Connection | Purpose |
|-------|-----------|---------|
| **CH1** | VBST test point | DRV2665 boost converter -- confirms device is active |
| **CH2** | Actuator output pin 4 | High-voltage output at the piezo -- the actual result |
| **CH3** | GPIO5 / R13 input side | Raw PWM signal before filter (312.5 kHz carrier) |
| **CH4** | PWM_FILTERED / IN+ net | Filtered analog signal after R13+C9, at DRV2665 input |

If only 2 scope channels are free, prioritize **CH3 = raw PWM** and **CH4 = filtered output at IN+**. These two let you see the filter working.

### Logic analyzer connections (existing)

Keep current wiring:
- A0 = SDA, A1 = SCL (I2C to DRV2665)
- A2 = MOSI, A3 = MISO, A4 = SCLK, A5 = SS (SPI to shift registers)
- A6 = POL (polarity toggle)

## Test Plan

### Phase 0: Flash and basic init

1. Build and flash firmware
2. Connect to REPL
3. Run `pz_actuator.init()` -- should print init messages without errors
4. Run `pz_actuator.is_running()` -- should return `False`

**MCP verification:**
```
mcp__micropython__exec: "import pz_actuator; pz_actuator.init()"
mcp__micropython__exec: "import pz_actuator; pz_actuator.is_running()"
```

### Phase 1: Analog path -- basic sine wave

**Test 1.1: 250 Hz analog sine**
```python
import pz_actuator
pz_actuator.init()
pz_actuator.set_frequency_analog(250)
pz_actuator.set_all(True)
pz_actuator.start()
```

**Scope checks (CH3 raw PWM, CH4 filtered):**
- CH3: 312.5 kHz PWM with varying duty cycle (DDS modulation visible)
- CH4: clean ~250 Hz sine wave, 0-3.3V swing, centered at ~1.65V
- CH1 (VBST): should show boost converter activity
- CH2 (actuator): high-voltage sine at actuator

**MCP scope verification:**
```
mcp__siglent-sds__configure_channel(channel=3, enabled=true, scale="1V", coupling="DC")
mcp__siglent-sds__configure_channel(channel=4, enabled=true, scale="1V", coupling="DC")
mcp__siglent-sds__configure_acquisition(timebase="1ms")    # ~4 cycles at 250Hz
mcp__siglent-sds__measure(channel=4, type="frequency")     # expect ~250 Hz
mcp__siglent-sds__measure(channel=4, type="pkpk")          # expect ~3.3V before amp
mcp__siglent-sds__screenshot()
```

**Test 1.2: Frequency sweep**
```python
import pz_actuator, time
pz_actuator.init()
pz_actuator.set_all(True)
for freq in [50, 100, 200, 250, 300, 400]:
    pz_actuator.set_frequency_analog(freq)
    pz_actuator.start()
    time.sleep(3)  # measure each
    pz_actuator.stop()
```

For each frequency, measure with scope and verify frequency matches target.

**Test 1.3: DC mode**
```python
pz_actuator.set_frequency_analog(0)
pz_actuator.start()
```

**Scope check:** CH4 should show flat DC at ~3.3V (100% duty after filter).

**Test 1.4: 10-bit resolution**
```python
pz_actuator.set_frequency_analog(250, resolution=10)
pz_actuator.start()
```

**Scope check:** CH3 carrier drops from 312.5 kHz to 78.1 kHz. CH4 sine should be slightly smoother (1024 vs 256 duty levels) but potentially more ripple at the lower carrier frequency.

### Phase 2: Digital path -- I2C FIFO sine

**Test 2.1: 250 Hz digital sine**
```python
pz_actuator.set_frequency_digital(250)
pz_actuator.set_all(True)
pz_actuator.start()
```

**Logic analyzer check (I2C):**
```
mcp__logic-analyzer__capture_and_decode(
    protocol="i2c",
    channels={"sda": 0, "scl": 1},
    sample_rate=1000000,
    duration=0.05
)
```

Expect: continuous I2C writes to address 0x59, register 0x0B (FIFO data), with signed 8-bit sine samples.

**Scope check:** CH2 (actuator output) should show sine wave. Note: digital path has known 6.83x frequency scaling -- actual output will be ~36.6 Hz for 250 Hz target.

**Test 2.2: Compare analog vs digital at same target frequency**

Run analog 250 Hz, measure actual frequency on scope. Then run digital 250 Hz, measure actual frequency. Document the difference (analog should be accurate, digital will be ~6.83x slower).

### Phase 3: Mode switching

**Test 3.1: Analog to digital switch**
```python
pz_actuator.set_frequency_analog(250)
pz_actuator.start()
# ... verify analog output ...
pz_actuator.stop()
pz_actuator.set_frequency_digital(100)
pz_actuator.start()
# ... verify digital output ...
pz_actuator.stop()
```

Verify: no crashes, clean transitions, DRV2665 goes to standby between modes.

**Test 3.2: Restart without reconfigure**
```python
pz_actuator.set_frequency_analog(250)
pz_actuator.start()
pz_actuator.stop()
pz_actuator.start()  # should restart same mode
```

Verify: second start() works without calling set_frequency again.

### Phase 4: Gain settings

**Test 4.1: Gain sweep**
```python
pz_actuator.set_frequency_analog(250)
pz_actuator.set_all(True)
for gain in [25, 50, 75, 100]:
    pz_actuator.start(gain=gain)
    # measure CH2 (actuator output) peak-to-peak
    time.sleep(2)
    pz_actuator.stop()
```

**Scope check (CH2 actuator output):**
- gain=25: ~50 Vpp
- gain=50: ~100 Vpp
- gain=75: ~150 Vpp
- gain=100: ~200 Vpp

(Actual voltages depend on input amplitude and boost voltage. Key thing is that higher gain = higher output.)

### Phase 5: Pin control during output

**Test 5.1: Individual pin activation**
```python
pz_actuator.set_frequency_analog(250)
pz_actuator.start()

pz_actuator.set_all(False)     # all off
pz_actuator.set_pin(4, True)   # only pin 4 (scope CH2)
# scope should show signal
pz_actuator.set_pin(4, False)
# scope should go flat
```

**Test 5.2: Pin cycling**
```python
pz_actuator.set_frequency_analog(250)
pz_actuator.start()
for i in range(20):
    pz_actuator.set_all(False)
    pz_actuator.set_pin(i, True)
    time.sleep_ms(200)
```

Logic analyzer (SPI): verify shift register updates for each pin change.

### Phase 6: Error handling

**Test 6.1: start() before set_frequency**
```python
pz_actuator.init()
pz_actuator.start()  # should raise RuntimeError
```

**Test 6.2: Invalid gain**
```python
pz_actuator.set_frequency_analog(250)
pz_actuator.start(gain=60)  # should raise ValueError
```

**Test 6.3: Invalid resolution**
```python
pz_actuator.set_frequency_analog(250, resolution=12)  # should raise ValueError
```

## Filter Quality Assessment

After Test 1.1, evaluate the RC filter performance:

| What to measure | Where | Expected | Action if bad |
|-----------------|-------|----------|---------------|
| Carrier ripple | CH4 (filtered) | < 50mV at 8-bit | Increase C9 to 220nF |
| Sine shape | CH4 (filtered) | Smooth, no steps | OK at 32kHz DDS rate |
| Frequency accuracy | CH4 frequency | Within 1% of target | Check DDS step calc |
| DC offset | CH4 mean | ~1.65V (half Vdd) | Expected for unipolar PWM |
| DC mode level | CH4 at freq=0 | ~3.3V flat | Full duty cycle |

## Test Execution Order

1. **Solder R13 + C9 + JP7** (bridge 1-2)
2. **Connect scope probes** (CH3 = GPIO5, CH4 = IN+ net)
3. **Flash firmware**
4. Run Phase 0 (init)
5. Run Phase 1 (analog path -- most important, first time testing filter)
6. Evaluate filter quality, swap components if needed
7. Run Phase 2 (digital path)
8. Run Phase 3-6 (mode switching, gain, pins, errors)
