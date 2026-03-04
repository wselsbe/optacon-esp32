# Music Player & Build Optimization Design

Date: 2026-03-04
Branch: claude/pwm-sheet-music-player-Uai7m

## 1. Smooth Music Playback (Live Frequency Updates)

### Problem

`play()` calls `pa.stop()` / `pa.start()` per note, putting the DRV2665 into standby and restarting the ISR. This causes audible clicks/pops between notes.

### Solution

Keep the DRV2665 on and ISR running for the entire song. Add a live frequency update path that modifies DDS state without stopping the timer.

### C module changes (`pwm.c`)

Add `pzd_pwm_set_frequency_live(hz, amplitude, waveform)`:
- Writes `s_phase_step`, `s_amplitude`, `s_waveform` directly
- Does NOT stop/restart the GPTimer or reconfigure LEDC
- Does NOT reset `s_phase_acc` (smooth phase continuity between notes)
- For silence (rest): set `s_amplitude = 0` → ISR outputs duty=128 (midpoint)
- Validates hz (1-500) and amplitude (0-128) same as `pzd_pwm_set_frequency`

### Python bindings (`pz_drive.c`)

Expose: `pz_drive.pwm_set_frequency_live(hz, amplitude=128, waveform=0)`

### PzActuator changes (`pz_drive_py.py`)

Add `set_frequency_live(hz, amplitude=100, waveform='sine')`:
- Maps amplitude 0-100 → 0-128
- Maps waveform name → int
- Calls `pz_drive.pwm_set_frequency_live()`
- No DRV2665 register writes needed (already configured)

### Extended note format

Tuples of varying length, backward compatible:

```python
(note, beats)                    # default amplitude
(note, beats, amplitude)         # 0-100 dynamics (pp=20, p=40, mf=60, f=80, ff=100)
(note, beats, amplitude, sweep)  # glissando/pitch bend to target note
("R", beats)                     # rest (amplitude → 0)
```

### Music player changes (`music.py`)

Rewrite `play()`:
1. `pa.start(gain=gain)` once at the beginning
2. For each note: `pa.set_frequency_live(freq, amplitude, waveform)`
3. For rests: `pa.set_frequency_live(current_freq, amplitude=0)`
4. For sweeps: set start freq via live update, then call `pz_drive.pwm_set_sweep(step_end, increment)` directly — the existing ISR-level sweep modifies `s_phase_step` each tick and auto-stops at target. No new C code needed for glissando.
5. `pa.stop()` once at the end
6. Suppress per-note debug prints (the `mp_printf` in `pzd_pwm_set_frequency_live` should be silent or minimal)

## 2. Selective Freezing (Build Optimization)

### Problem

`freeze(".")` in `manifest.py` freezes ALL `python/*.py` into firmware. Any Python file change triggers a firmware rebuild (~30s incremental). Files that change frequently don't need frozen bytecode performance.

### Solution

Split Python files into frozen (compiled into firmware) and filesystem (uploaded via mpremote).

### Frozen (in manifest.py) — core drivers, rarely change

- `pz_drive_py.py` — hardware orchestrator
- `drv2665.py` — I2C register driver
- `shift_register.py` — SPI shift register driver
- `main.py` — boot script
- `microdot/` — web framework (third-party, never changes)

### Filesystem (uploaded via mpremote) — change frequently

- `music.py` — song data and player
- `web_server.py` — web UI routes
- `wifi.py` — WiFi connection helper

### manifest.py changes

```python
include("$(PORT_DIR)/boards/manifest.py")

# Core drivers (frozen into firmware)
freeze(".", ("pz_drive_py.py", "drv2665.py", "shift_register.py", "main.py"))
module("microdot", base_path=".")
```

### Flash skill updates

Add to Step 1 decision logic:
- If ONLY filesystem Python files changed (`music.py`, `web_server.py`, `wifi.py`): skip build, just copy via mpremote
- If frozen files or C modules changed: full build + flash + copy filesystem files

Add new Step 4a (after flash, before verify):
```bash
mpremote cp python/music.py python/web_server.py python/wifi.py :
```

Add a fast-path shortcut: when no build is needed, go directly to mpremote copy + soft reset (no bootloader, no flash).

## 3. Additional Songs

Add to `music.py` as named constants. Selected for recognizability on single-voice piezo:

- **FUR_ELISE** — Beethoven, iconic opening ~8 measures
- **ODE_TO_JOY** — Beethoven, simple stepwise melody
- **IMPERIAL_MARCH** — Star Wars, strong distinctive intervals
- **NOKIA_TUNE** — Gran Vals excerpt, universally recognized, short

Each defined as list of extended tuples with dynamics where appropriate (e.g., Für Elise pp opening, Imperial March ff).

Suggested defaults per song exported alongside the data:
```python
SONGS = {
    "clair_de_lune": (CLAIR_DE_LUNE, 200, 50),   # (data, bpm, gain)
    "fur_elise":     (FUR_ELISE, 160, 50),
    "ode_to_joy":    (ODE_TO_JOY, 200, 75),
    "imperial_march":(IMPERIAL_MARCH, 208, 100),
    "nokia":         (NOKIA_TUNE, 280, 75),
}
```

Add convenience: `play_song(name)` that looks up from SONGS dict.
