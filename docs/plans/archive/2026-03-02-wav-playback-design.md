# WAV Audio Playback via Analog PWM Path

## Problem

Need to play arbitrary audio (voice messages as .wav files) through the DRV2665 piezo driver using the existing ESP32-S3 hardware and RC filter.

## Design

### Signal Path

```
WAV file → Python bytearray → C ISR (32 kHz) → PWM GPIO5 → RC filter (4.08 kHz cutoff) → DRV2665 IN+
```

The existing 32 kHz DDS ISR in `pwm.c` gets a new mode: instead of computing waveforms from a phase accumulator, it reads samples from a provided buffer using a fixed-point phase accumulator to handle any sample rate.

### WAV Parsing (Python)

Python reads the 44-byte WAV header to extract:
- `sample_rate` (bytes 24-27, little-endian) — e.g., 8000, 16000, 44100
- `bits_per_sample` (bytes 34-35) — 8 or 16
- `num_channels` (bytes 22-23) — 1 (mono) or 2 (stereo)

Conversion to 8-bit mono:
- 16-bit → 8-bit: shift right by 8, add 128 (convert signed to unsigned)
- Stereo → mono: take every other sample (discard right channel)

### Resampling (C ISR)

The ISR runs at a fixed 32 kHz. A fixed-point phase accumulator steps through the buffer at the WAV's native rate:

```
step = (sample_rate << 16) / 32000
position += step           // each ISR tick
sample = buf[position >> 16]
```

Linear interpolation between adjacent samples for smoothness. This handles any WAV sample rate (8 kHz to 44.1 kHz) with no extra RAM.

### Playback Behavior

- Play once (default): ISR stops and enters standby when buffer exhausts
- Loop: wraps position to start of buffer, plays indefinitely until `stop()`

### API

```python
pa.play_wav("/audio/message.wav", loop=False)  # plays once
pa.play_wav("/audio/tone.wav", loop=True)       # loops until stop()
pa.stop()
```

### Files to Modify

| File | Changes |
|------|---------|
| `modules/pz_drive/pwm.c` | Add sample buffer mode to ISR with phase accumulator resampling. New `pzd_pwm_play_samples(buf, len, sample_rate, loop)`. |
| `modules/pz_drive/pz_drive.c` | Add Python binding `pwm_play_samples(buf, sample_rate, loop)` |
| `modules/pz_drive/pz_drive.h` | Declare `pzd_pwm_play_samples()` |
| `python/pz_actuator_py.py` | Add `play_wav(path, loop=False)` — reads file, parses header, converts to 8-bit mono, calls C |

### Constraints

- ~200 KB free flash for WAV files
- WAV must be PCM format (not compressed). 8-bit or 16-bit, mono or stereo.
- RC filter cutoff 4.08 kHz limits audio bandwidth to voice band
- Playback always at 32 kHz ISR rate, resampled from native rate
