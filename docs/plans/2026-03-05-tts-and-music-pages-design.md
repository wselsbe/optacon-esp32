# Text-to-Speech & Music Pages Design

**Goal:** Add on-device text-to-speech using SAM (Software Automatic Mouth) as a C module, plus two new web pages for speech and sheet music playback.

**Architecture:** SAM's C code (~39 KB) compiles as a MicroPython C user module. It converts English text to phonemes, then synthesizes formant-based 8-bit PCM at 22,050 Hz. The existing `pwm_play_samples()` ISR handles playback with automatic resampling to 32 kHz. Two new web pages (`/speech`, `/music`) provide browser-based controls via WebSocket commands.

## SAM C Module

### What SAM does

SAM (1982) is a formant synthesizer. The pipeline:

1. **Reciter** — English text to phoneme string (rule-based, ~200 rules)
2. **Parser** — phoneme string to formant parameter tables (frequency, amplitude, duration per phoneme)
3. **Renderer** — formant parameters to PCM audio: `A1*sin(f1*t) + A2*sin(f2*t) + A3*rect(f3*t)`

The rectangular wave component is what gives SAM its characteristic buzzy/square-wave quality — well suited to piezo output.

### Source

Port from [s-macke/SAM](https://github.com/s-macke/SAM) (C, MIT-like license). Strip SDL audio output and file I/O — keep only the in-memory synthesis path.

### Module location

`modules/sam/` — compiled as a MicroPython C user module alongside `pz_drive` and `board_utils`.

### Python API

```python
import sam

# Blocking — renders PCM, plays via pwm_play_samples, waits for completion
sam.say("hello world")
sam.say("wifi connected", speed=100, pitch=64)

# Non-blocking — returns raw 8-bit unsigned PCM bytearray at 22050 Hz
buf = sam.render("hello world")
buf = sam.render("test", speed=72, pitch=64, mouth=128, throat=128)
```

**Parameters:**
- `text` — English text string (SAM's reciter handles text-to-phoneme)
- `speed` — speech rate (default 72, lower = slower)
- `pitch` — voice pitch (default 64, 0-255)
- `mouth` — mouth shape parameter (default 128)
- `throat` — throat shape parameter (default 128)

**`say()` implementation:**
1. Call SAM's C render function → get PCM buffer
2. Start PWM ISR if not running, set gain
3. Call `pz_drive.pwm_play_samples(buf, 22050)`
4. Poll `pwm_is_sample_done()` with `mp_handle_pending(True)` so Ctrl-C works
5. Return when done

**Memory:**
- Flash: ~39 KB for SAM C code + phoneme tables
- RAM: temporary PCM buffer during synthesis (~22 KB per second of speech). A typical short sentence (2-3 seconds) uses ~50-70 KB. ESP32-S3 has ~300 KB free heap.
- Buffer is freed after `say()` returns or when caller discards the `render()` result.

## WebSocket Commands

### Speech

```json
{"cmd": "say", "text": "hello world", "speed": 72, "pitch": 64}
```

Handler in `web_server.py` calls `sam.say()`. Response sent after speech completes (blocking).

### Music

```json
{"cmd": "play_music", "notes": "C4:4 D4:8 E4:2 R:4 G4:1", "bpm": 120, "waveform": "square"}
{"cmd": "stop_music"}
```

Handler parses the note string and plays via `PzActuator` using the same approach as `music.py`. Notes use the format `NOTE:DURATION` where duration is in beats (4=quarter, 8=eighth, etc.) and `R` is a rest.

## Web Pages

### `/speech` — Text-to-Speech

- Text input field for arbitrary English text
- Speed slider (range 1-255, default 72)
- Pitch slider (range 1-255, default 64)
- SPEAK button — sends WebSocket `say` command, disabled while speaking
- Status text showing "Speaking..." / idle

### `/music` — Sheet Music Player

- Text area for note notation (multi-line, e.g. `C4:4 D4:4 E4:2`)
- Tempo input (BPM, default 120)
- Waveform selector (sine / triangle / square)
- Gain selector (25 / 50 / 75 / 100 Vpp)
- PLAY / STOP buttons
- Status text showing "Playing..." / idle

### Navigation

All pages get links to `/speech` and `/music` in the footer nav, alongside existing Control Panel, WiFi, Updates links.

## Files

| File | Action | Description |
|------|--------|-------------|
| `modules/sam/` | Create | SAM C source + MicroPython bindings |
| `modules/sam/micropython.cmake` | Create | CMake integration |
| `modules/micropython.cmake` | Modify | Add `include(sam/micropython.cmake)` |
| `python/web_server.py` | Modify | Add `say`, `play_music`, `stop_music` commands + routes |
| `web/speech.html` | Create | TTS web page |
| `web/music.html` | Create | Sheet music web page |
| `web/index.html` | Modify | Add nav links to speech/music |
| `web/wifi.html` | Modify | Add nav links |
| `web/update.html` | Modify | Add nav links |
| `web/docs.html` | Modify | Add nav links |
| `python/manifest.py` | No change | SAM is C module, not frozen Python |

## Constraints

- **Firmware size:** SAM adds ~39 KB. Current firmware is 1,375 KB with 197 KB free in OTA slot. Fits comfortably.
- **RAM:** Speech buffer is transient. Longest practical utterance ~5 seconds (~110 KB). Beyond that, SAM could fail to allocate. Not a problem for typical use.
- **Concurrency:** `say()` blocks — no simultaneous speech + music. The ISR can only do one thing at a time (DDS or sample playback, not both).
- **No streaming:** SAM renders the entire utterance to a buffer before playback starts. There will be a brief pause between calling `say()` and hearing audio (render time ~100-500 ms for a sentence).
