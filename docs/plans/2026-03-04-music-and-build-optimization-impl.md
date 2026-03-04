# Music Player & Build Optimization Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Smooth music playback (no start/stop per note), selective Python freezing for faster dev iteration, flash skill update, and additional songs.

**Architecture:** Add a `pzd_pwm_set_frequency_live()` C function that updates DDS state without stopping the ISR. Rewrite `music.py` to use it. Split `manifest.py` into frozen (core drivers) vs filesystem (app-level) modules.

**Tech Stack:** C (ESP-IDF, MicroPython C API), MicroPython, mpremote

---

### Task 1: Add `pzd_pwm_set_frequency_live()` to C module

**Files:**
- Modify: `modules/pz_drive/pz_drive.h:33-41` (add declaration)
- Modify: `modules/pz_drive/pwm.c:464-487` (add function after `pzd_pwm_start`)
- Modify: `modules/pz_drive/pz_drive.c:122-137` (add Python binding)
- Modify: `modules/pz_drive/pz_drive.c:241-264` (add to module table)

**Step 1: Add declaration to `pz_drive.h`**

After line 41 (`bool pzd_pwm_is_sweep_done(void);`), add:

```c
void pzd_pwm_set_frequency_live(int hz, int amplitude, int waveform);
```

**Step 2: Add implementation to `pwm.c`**

After `pzd_pwm_start()` (after line 482), add:

```c
void pzd_pwm_set_frequency_live(int hz, int amplitude, int waveform) {
    if (!s_running) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("pwm not running"));
    }
    if (hz < 0 || hz > 500) {
        mp_raise_ValueError(MP_ERROR_TEXT("hz must be 0-500"));
    }
    if (amplitude < 0) amplitude = 0;
    if (amplitude > 128) amplitude = 128;
    if (waveform < 0 || waveform > 2) waveform = WAVEFORM_SINE;

    // Update DDS state — ISR reads these each tick, no lock needed
    s_phase_step = (hz == 0) ? 0 : (uint32_t)(((uint64_t)hz << 32) / PWM_SAMPLE_RATE_HZ);
    s_amplitude = (uint8_t)amplitude;
    s_waveform = (uint8_t)waveform;
    // Do NOT reset s_phase_acc — preserves phase continuity
}
```

**Step 3: Add Python binding to `pz_drive.c`**

After `pz_drive_pwm_set_frequency_obj` (line 122), add:

```c
// ── pwm_set_frequency_live(hz, amplitude=128, waveform=0) ────────────
static mp_obj_t pz_drive_pwm_set_frequency_live(size_t n_args, const mp_obj_t *pos_args,
                                                mp_map_t *kw_args) {
    enum { ARG_hz, ARG_amplitude, ARG_waveform };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_hz, MP_ARG_REQUIRED | MP_ARG_INT, {0}},
        {MP_QSTR_amplitude, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 128}},
        {MP_QSTR_waveform, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    pzd_pwm_set_frequency_live(args[ARG_hz].u_int, args[ARG_amplitude].u_int,
                               args[ARG_waveform].u_int);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_drive_pwm_set_frequency_live_obj, 1,
                                  pz_drive_pwm_set_frequency_live);
```

Add to module globals table (after `pwm_set_frequency` entry, line 253):

```c
    {MP_ROM_QSTR(MP_QSTR_pwm_set_frequency_live), MP_ROM_PTR(&pz_drive_pwm_set_frequency_live_obj)},
```

**Step 4: Build to verify compilation**

Run: `./scripts/build.sh`
Expected: Successful build with no errors.

**Step 5: Commit**

```bash
git add modules/pz_drive/pz_drive.h modules/pz_drive/pwm.c modules/pz_drive/pz_drive.c
git commit -m "feat(pz_drive): add pwm_set_frequency_live for smooth note transitions"
```

---

### Task 2: Add `set_frequency_live()` to PzActuator

**Files:**
- Modify: `python/pz_drive_py.py:134` (add method before `start()`)

**Step 1: Add method to PzActuator**

Before `def start()` (line 136), add:

```python
    def set_frequency_live(self, hz, amplitude=100, waveform="sine"):
        """Update frequency/amplitude while ISR is running (no restart).

        Use during music playback for smooth note transitions.
        For rests, set amplitude=0 (ISR outputs silence at midpoint).
        """
        if waveform not in self.WAVEFORMS:
            raise ValueError("waveform must be 'sine', 'triangle', or 'square'")
        amp_internal = (amplitude * 128 + 50) // 100
        pz_drive.pwm_set_frequency_live(
            hz,
            amplitude=amp_internal,
            waveform=self.WAVEFORMS[waveform],
        )
        self._frequency = hz
        self._amplitude = amplitude
        self._waveform_name = waveform
```

**Step 2: Commit**

```bash
git add python/pz_drive_py.py
git commit -m "feat(pz_drive_py): add set_frequency_live method"
```

---

### Task 3: Rewrite music.py player with live updates and extended format

**Files:**
- Modify: `python/music.py:69-131` (rewrite `play()`)
- Modify: `python/music.py:203` (add `play_song()` convenience)

**Step 1: Rewrite `play()` function**

Replace the entire `play()` function (lines 69-131) with:

```python
_PWM_SAMPLE_RATE = 32000


def play(song, bpm=72, gain=100, waveform="sine", amplitude=100,
         staccato_ratio=0.9, loop=False):
    """Play a song: list of note tuples.

    Note format (variable-length tuples, backward compatible):
        (note, beats)                    - play at default amplitude
        (note, beats, amplitude)         - play with dynamics (0-100)
        (note, beats, amplitude, sweep)  - glissando to target note
        ("R", beats)                     - rest (silence)

    Args:
        song: list of note tuples
        bpm: tempo in beats per minute (default 72)
        gain: DRV2665 gain - 25, 50, 75, or 100 Vpp
        waveform: 'sine', 'triangle', or 'square'
        amplitude: default amplitude 0-100 (used when tuple has no amplitude)
        staccato_ratio: fraction of note duration to sound (0.0-1.0)
        loop: if True, repeat until KeyboardInterrupt
    """
    import pz_drive
    from pz_drive_py import PzActuator

    pa = PzActuator()
    beat_ms = int(60_000 / bpm)
    wf_int = pa.WAVEFORMS.get(waveform, 0)

    # Configure initial frequency and start once
    pa.set_frequency_analog(220, amplitude=amplitude, waveform=waveform)
    pa.start(gain=gain)

    try:
        while True:
            for entry in song:
                note = entry[0]
                beats = entry[1]
                note_amp = entry[2] if len(entry) > 2 else amplitude
                sweep_target = entry[3] if len(entry) > 3 else None

                dur_ms = int(beat_ms * beats)
                sound_ms = int(dur_ms * staccato_ratio)
                gap_ms = dur_ms - sound_ms

                is_rest = note is None or note == "R"

                if is_rest:
                    pa.set_frequency_live(1, amplitude=0, waveform=waveform)
                    time.sleep_ms(dur_ms)
                else:
                    freq = note_freq(note)
                    if freq < 1:
                        freq = 1
                    elif freq > 500:
                        freq = 500
                    freq = int(freq)

                    if sweep_target is not None:
                        # Start at current note, sweep to target
                        pa.set_frequency_live(freq, amplitude=note_amp,
                                              waveform=waveform)
                        target_freq = int(note_freq(sweep_target))
                        if target_freq < 1:
                            target_freq = 1
                        elif target_freq > 500:
                            target_freq = 500
                        total_ticks = sound_ms * (_PWM_SAMPLE_RATE // 1000)
                        step_end = (target_freq << 32) // _PWM_SAMPLE_RATE
                        step_start = (freq << 32) // _PWM_SAMPLE_RATE
                        if total_ticks > 0 and step_start != step_end:
                            increment = (step_end - step_start) // total_ticks
                            pz_drive.pwm_set_sweep(step_end, increment)
                    else:
                        pa.set_frequency_live(freq, amplitude=note_amp,
                                              waveform=waveform)

                    time.sleep_ms(sound_ms)

                    if gap_ms > 0:
                        pa.set_frequency_live(freq, amplitude=0,
                                              waveform=waveform)
                        time.sleep_ms(gap_ms)

            if not loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        pa.stop()
```

**Step 2: Add `play_song()` and SONGS dict**

After the last song data constant (end of file), add:

```python
SONGS = {
    "clair_de_lune": (CLAIR_DE_LUNE, 200, 50),
}


def play_song(name, **kwargs):
    """Play a named song. Extra kwargs override defaults."""
    if name not in SONGS:
        raise ValueError("unknown song: " + name + " (available: " +
                         ", ".join(SONGS.keys()) + ")")
    data, default_bpm, default_gain = SONGS[name]
    kwargs.setdefault("bpm", default_bpm)
    kwargs.setdefault("gain", default_gain)
    play(data, **kwargs)
```

**Step 3: Commit**

```bash
git add python/music.py
git commit -m "feat(music): smooth playback with live freq updates and extended note format"
```

---

### Task 4: Add songs

**Files:**
- Modify: `python/music.py` (add song constants before `SONGS` dict)

**Step 1: Add Für Elise**

After `CLAIR_DE_LUNE`, add:

```python
# ---------------------------------------------------------------------------
# Für Elise — Beethoven (opening theme, 3/8 time)
# Beats in eighth-note units. bpm=240 (eighth) ≈ 80 bpm (quarter).
# ---------------------------------------------------------------------------

FUR_ELISE = [
    # A section (repeated)
    ("E5", 1), ("D#5", 1),
    ("E5", 1), ("D#5", 1), ("E5", 1), ("B4", 1), ("D5", 1), ("C5", 1),
    ("A4", 3),
    ("R", 1), ("C4", 1), ("E4", 1),
    ("A4", 3),
    ("R", 1), ("E4", 1), ("G#4", 1),
    ("B4", 3),
    ("R", 1), ("E4", 1), ("E5", 1), ("D#5", 1),
    ("E5", 1), ("D#5", 1), ("E5", 1), ("B4", 1), ("D5", 1), ("C5", 1),
    ("A4", 3),
    ("R", 1), ("C4", 1), ("E4", 1),
    ("A4", 3),
    ("R", 1), ("E4", 1), ("C5", 1), ("B4", 1),
    ("A4", 3),
]
```

**Step 2: Add Ode to Joy**

```python
# ---------------------------------------------------------------------------
# Ode to Joy — Beethoven (main theme, 4/4 time)
# Beats in quarter-note units. bpm=120.
# ---------------------------------------------------------------------------

ODE_TO_JOY = [
    ("E4", 1), ("E4", 1), ("F4", 1), ("G4", 1),
    ("G4", 1), ("F4", 1), ("E4", 1), ("D4", 1),
    ("C4", 1), ("C4", 1), ("D4", 1), ("E4", 1),
    ("E4", 1.5), ("D4", 0.5), ("D4", 2),
    ("E4", 1), ("E4", 1), ("F4", 1), ("G4", 1),
    ("G4", 1), ("F4", 1), ("E4", 1), ("D4", 1),
    ("C4", 1), ("C4", 1), ("D4", 1), ("E4", 1),
    ("D4", 1.5), ("C4", 0.5), ("C4", 2),
    ("D4", 1), ("D4", 1), ("E4", 1), ("C4", 1),
    ("D4", 1), ("E4", 0.5), ("F4", 0.5), ("E4", 1), ("C4", 1),
    ("D4", 1), ("E4", 0.5), ("F4", 0.5), ("E4", 1), ("D4", 1),
    ("C4", 1), ("D4", 1), ("G3", 2),
    ("E4", 1), ("E4", 1), ("F4", 1), ("G4", 1),
    ("G4", 1), ("F4", 1), ("E4", 1), ("D4", 1),
    ("C4", 1), ("C4", 1), ("D4", 1), ("E4", 1),
    ("D4", 1.5), ("C4", 0.5), ("C4", 2),
]
```

**Step 3: Add Imperial March**

```python
# ---------------------------------------------------------------------------
# Imperial March — John Williams (main theme)
# Beats in quarter-note units. bpm=104.
# ---------------------------------------------------------------------------

IMPERIAL_MARCH = [
    ("G4", 2, 90), ("G4", 2, 90), ("G4", 2, 90),
    ("Eb4", 1.5, 80), ("Bb4", 0.5, 70),
    ("G4", 2, 90), ("Eb4", 1.5, 80), ("Bb4", 0.5, 70),
    ("G4", 4, 100),
    ("D5", 2, 90), ("D5", 2, 90), ("D5", 2, 90),
    ("Eb5", 1.5, 80), ("Bb4", 0.5, 70),
    ("Gb4", 2, 90), ("Eb4", 1.5, 80), ("Bb4", 0.5, 70),
    ("G4", 4, 100),
]
```

**Step 4: Add Nokia tune**

```python
# ---------------------------------------------------------------------------
# Nokia Tune — Gran Vals (Francisco Tárrega), 3/4 time
# Beats in eighth-note units. bpm=280.
# ---------------------------------------------------------------------------

NOKIA_TUNE = [
    ("E5", 2), ("D5", 2),
    ("F#4", 4), ("G#4", 4),
    ("C#5", 2), ("B4", 2),
    ("D4", 4), ("E4", 4),
    ("B4", 2), ("A4", 2),
    ("C#4", 4), ("E4", 4),
    ("A4", 8),
]
```

**Step 5: Update SONGS dict**

```python
SONGS = {
    "clair_de_lune": (CLAIR_DE_LUNE, 200, 50),
    "fur_elise": (FUR_ELISE, 240, 50),
    "ode_to_joy": (ODE_TO_JOY, 120, 75),
    "imperial_march": (IMPERIAL_MARCH, 104, 100),
    "nokia": (NOKIA_TUNE, 280, 75),
}
```

**Step 6: Commit**

```bash
git add python/music.py
git commit -m "feat(music): add Für Elise, Ode to Joy, Imperial March, Nokia tune"
```

---

### Task 5: Selective freezing in manifest.py

**Files:**
- Modify: `python/manifest.py`

**Step 1: Replace `freeze(".")` with selective freezing**

Replace entire file with:

```python
# Standard MicroPython frozen modules
include("$(PORT_DIR)/boards/manifest.py")

# Core drivers (frozen into firmware — rarely change)
freeze(".", ("pz_drive_py.py", "drv2665.py", "shift_register.py", "main.py"))

# Third-party (frozen — never changes)
module("microdot", base_path=".")
```

**Step 2: Verify build still works**

Run: `./scripts/build.sh`
Expected: Successful build. `music.py`, `web_server.py`, `wifi.py` should NOT appear in the `MPY` lines.

**Step 3: Commit**

```bash
git add python/manifest.py
git commit -m "build: selective freezing — only core drivers frozen, app modules go to filesystem"
```

---

### Task 6: Update flash skill for filesystem Python files

**Files:**
- Modify: `.claude/skills/flash/SKILL.md`

**Step 1: Update the flash skill**

Key changes to the skill document:

1. **Step 1 (build decision)**: Add a third category:
   - `modules/**/*.c`, `modules/**/*.h`, frozen Python (`pz_drive_py.py`, `drv2665.py`, `shift_register.py`, `main.py`), `manifest.py`, `microdot/`, build config → **full build needed**
   - Filesystem Python only (`music.py`, `web_server.py`, `wifi.py`) → **skip build, copy via mpremote only**
   - `web/`, config files → **skip build, copy via mpremote only**

2. **Add new Step 4a** (after flash, before verify): Upload filesystem Python files:
   ```bash
   mpremote cp python/music.py python/web_server.py python/wifi.py :
   ```

3. **Add Fast Path section**: When only filesystem files changed:
   - Skip build entirely
   - Skip bootloader entry
   - Copy files via mpremote directly: `mpremote cp <files> :`
   - Soft reset: `mcp__micropython__soft_reset()`
   - Verify boot

**Step 2: Commit**

```bash
git add .claude/skills/flash/SKILL.md
git commit -m "docs(flash): update skill for selective freezing and mpremote fast path"
```

---

### Task 7: Flash, test, and verify

**Step 1: Build and flash firmware** (C module + frozen Python changed)

Use `/flash` skill.

**Step 2: Upload filesystem Python files**

```bash
mpremote cp python/music.py python/web_server.py python/wifi.py :
```

**Step 3: Test smooth playback**

```python
from music import play_song
play_song("clair_de_lune")     # should sound smooth, no clicks
play_song("fur_elise")
play_song("imperial_march")
play_song("nokia")
play_song("ode_to_joy")
```

**Step 4: Test extended note format (dynamics)**

```python
from music import play
# Crescendo test
test = [("C4", 2, 20), ("C4", 2, 40), ("C4", 2, 60), ("C4", 2, 80), ("C4", 2, 100)]
play(test, bpm=120, gain=75)
```

**Step 5: Test filesystem-only update (fast path)**

Edit `music.py` on the host, then:
```bash
mpremote cp python/music.py :
```
Then soft reset and verify the change took effect without a full reflash.

**Step 6: Commit any fixes**

```bash
git add -A
git commit -m "fix: adjustments from testing"
```
