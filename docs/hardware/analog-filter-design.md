# Analog Input Filter Design — DRV2665 IN+/IN-

**Date:** 2026-02-25
**PCB:** Optacon Motherboard (proj_kicad/motherboard)
**Schematic sheet:** "low pass filter.kicad_sch"

## Purpose

Convert a digital PWM signal from the ESP32-S3 (GPIO5) into a smooth analog sine wave suitable for the DRV2665/DRV2700 analog input (IN+). The filter removes the high-frequency PWM carrier while passing the desired signal band (DC to 400 Hz).

## PCB Filter Options

The motherboard has two signal paths selectable via solder jumper JP7:

### Path 1: Passive RC (JP7 pads 1-2) — RECOMMENDED

Simple single-pole RC low-pass filter.

```
GPIO5 ──► R13 ──┬── JP7(1-2) ──► PWM_FILTERED ──► IN+
                 │
                C9
                 │
                GND
```

**Components:**

| Ref | Value   | Footprint | Role |
|-----|---------|-----------|------|
| R13 | 390 Ω   | 0603      | Series resistance |
| C9  | 0.1 µF  | 0603      | Shunt to ground |
| JP7 | bridge 1-2 | SolderJumper_3 | Path select |

**Characteristics:**
- Cutoff: fc = 1/(2π × 390 × 100nF) = **4.08 kHz**
- Order: 1st (single pole, -20 dB/decade)
- DC pass: yes (required for "fully on" mode)
- Output impedance: 390 Ω (fine for high-Z DRV2665 analog inputs)

### Path 2: Active Multi-Stage (JP7 pads 2-3)

Three op-amp stages using LM324 (U4). Original design, never tested.

```
GPIO5 ──► R6 ──┬── U4A ──► R8 ── U4B ──► R9 ── R10 ── U4C ──► JP7(2-3) ──► PWM_FILTERED
               R7            C5         C6                C7
               │              │          │                 │
              GND            GND        GND            (feedback)
```

**Components (all 0603 passives, TSSOP-14 for U4):**

| Ref | Value    | Role |
|-----|----------|------|
| U4  | LM324    | Quad op-amp (TSSOP-14) |
| R6  | 12.4 kΩ  | Input to U4A inverting input |
| R7  | 6.19 kΩ  | Input divider to GND |
| R8  | 4.02 kΩ  | Inter-stage to U4B |
| R9  | 1.21 kΩ  | Inter-stage to U4C |
| R10 | 2.74 kΩ  | Inter-stage to U4C |
| C5  | 10 nF    | Shunt at U4A output |
| C6  | 10 nF    | U4B feedback/shunt |
| C7  | 47 nF    | U4C feedback |
| C8  | 10 nF    | VDD bypass for U4 |

**Also on board but separate circuit (U4D section):**

| Ref | Value   | Role |
|-----|---------|------|
| R11 | 100 kΩ  | U4D feedback |
| R12 | 100 kΩ  | U4D input |

Status: untested, designed for steeper rolloff if passive path proves insufficient.

## Design Rationale — Why Passive RC Is Enough

**Signal band:** DC to 400 Hz
**PWM carrier:** 312.5 kHz (8-bit) or 78.1 kHz (10-bit)

| Parameter | 8-bit / 312.5 kHz | 10-bit / 78.1 kHz |
|-----------|--------------------|--------------------|
| Carrier/signal ratio | 781:1 | 195:1 |
| Filter attenuation at carrier | -37.7 dB | -25.6 dB |
| Residual ripple (on 1V signal) | ~13 mV | ~52 mV |
| Signal loss at 400 Hz | -0.04 dB | -0.04 dB |

The 10 nF piezo actuators provide additional high-frequency attenuation. Being mechanical devices, they cannot physically respond to frequencies above a few kHz, further suppressing any residual carrier.

## Alternative Component Values

If the default values don't perform well, the same 0603 footprints accept different values:

### Lower cutoff (more filtering, less headroom above 400 Hz)

| R13 | C9 | fc | Atten @ 312.5 kHz | Atten @ 78.1 kHz | Loss @ 400 Hz |
|-----|----|----|--------------------|--------------------|---------------|
| 390 Ω | 0.22 µF | 1.86 kHz | -44.5 dB | -32.5 dB | -0.2 dB |
| 1 kΩ | 0.1 µF | 1.59 kHz | -45.9 dB | -33.8 dB | -0.3 dB |
| 1 kΩ | 0.22 µF | 723 Hz | -52.7 dB | -40.7 dB | -1.4 dB |

### Higher cutoff (less filtering, flatter passband)

| R13 | C9 | fc | Atten @ 312.5 kHz | Atten @ 78.1 kHz | Loss @ 400 Hz |
|-----|----|----|--------------------|--------------------|---------------|
| 390 Ω | 47 nF | 8.7 kHz | -31.1 dB | -19.1 dB | -0.01 dB |
| 220 Ω | 0.1 µF | 7.2 kHz | -32.7 dB | -20.7 dB | -0.01 dB |

### Two-stage passive (if one pole isn't enough)

Not directly supported by the current PCB passive path (only R13 + C9 footprints available). Would require the active path footprints with modified values, or a bodge wire.

## Recommendations

1. **Start with R13=390 Ω, C9=0.1 µF, 8-bit PWM** — test on the scope
2. If too much ripple: switch to R13=1 kΩ, C9=0.1 µF (fc=1.59 kHz)
3. If that's still not enough: fall back to the active filter path (populate U4 + surrounding components)
4. If more ripple is acceptable: try 10-bit for better sine resolution at the cost of more ripple

## Test Points

- **TP1** — near PWM_IN (raw PWM signal)
- **TP2** — near PWM_FILTERED output (filtered analog)
- **TP3** — GND reference
