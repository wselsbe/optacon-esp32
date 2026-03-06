"""Sheet music player for piezo actuators via PWM.

Plays sequences of notes defined as tuples with optional dynamics and sweeps.
Uses the analog PWM path through PzDrive with live frequency updates
(no start/stop per note) for smooth playback.

Note format (variable-length tuples, backward compatible):
    (note, beats)                    - play at default amplitude
    (note, beats, amplitude)         - play with dynamics (0-100)
    (note, beats, amplitude, sweep)  - glissando to target note
    ("R", beats)                     - rest (silence)

Usage:
    from music import play_song

    play_song("super_mario")               # play by name
    play_song("tetris", gain=50)           # quieter

    # Custom song with dynamics
    my_song = [
        ("C4", 1, 60), ("E4", 1, 80), ("G4", 1, 100), ("C5", 2),
        ("R", 1),
        ("G4", 1), ("E4", 1), ("C4", 2),
    ]
    play(my_song, bpm=120)
"""

import time

# ---------------------------------------------------------------------------
# Note-to-frequency table (A4 = 440 Hz, equal temperament)
# Covers C2–B6; extend if needed.
# ---------------------------------------------------------------------------

_NOTE_NAMES = ("C", "Db", "D", "Eb", "E", "F", "Gb", "G", "Ab", "A", "Bb", "B")

# Build lookup: note_name -> frequency (Hz)
# MIDI note 69 = A4 = 440 Hz; MIDI note 0 = C-1
NOTES = {}
for _oct in range(2, 7):
    for _i, _name in enumerate(_NOTE_NAMES):
        _midi = (_oct + 1) * 12 + _i  # C4 = MIDI 60
        _freq = 440.0 * (2.0 ** ((_midi - 69) / 12.0))
        NOTES[_name + str(_oct)] = round(_freq, 2)

# Enharmonic aliases
_ALIASES = {
    "C#": "Db", "D#": "Eb", "E#": "F", "F#": "Gb",
    "G#": "Ab", "A#": "Bb", "B#": "C",
    "Cb": "B", "Fb": "E",
}
for _oct in range(2, 7):
    for _sharp, _flat in _ALIASES.items():
        _target_oct = _oct
        if _sharp == "B#":
            _target_oct = _oct + 1
        elif _sharp == "Cb":
            _target_oct = _oct - 1
        _key = _sharp + str(_oct)
        _target = _flat + str(_target_oct)
        if _target in NOTES and _key not in NOTES:
            NOTES[_key] = NOTES[_target]


def note_freq(name):
    """Return frequency in Hz for a note name like 'C4', 'Db5', 'F#3'."""
    if name in NOTES:
        return NOTES[name]
    raise ValueError("unknown note: " + name)


# ---------------------------------------------------------------------------
# Player
# ---------------------------------------------------------------------------

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
        gain: DRV2665 gain — 25, 50, 75, or 100 Vpp
        waveform: 'sine', 'triangle', or 'square'
        amplitude: default amplitude 0-100 (used when tuple has no amplitude)
        staccato_ratio: fraction of note duration to sound (0.0-1.0)
        loop: if True, repeat until KeyboardInterrupt
    """
    import pz_drive
    from pz_drive_py import PzDrive

    pa = PzDrive()
    from shift_register import ShiftRegister
    sr = ShiftRegister()
    sr.set_all(True)
    beat_ms = int(60_000 / bpm)

    # Configure initial frequency and start once
    pa.set_frequency_analog(220, amplitude=amplitude, waveform=waveform)
    pa.start(gain=gain)

    try:
        prev_freq = 0
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
                    prev_freq = 0
                else:
                    freq = note_freq(note)
                    if freq < 1:
                        freq = 1
                    elif freq > 1000:
                        freq = 1000
                    freq = int(freq)

                    # Re-articulate repeated notes with a brief amplitude dip
                    if freq == prev_freq:
                        pa.set_frequency_live(
                            freq, amplitude=0, waveform=waveform
                        )

                    if sweep_target is not None:
                        pa.set_frequency_live(
                            freq, amplitude=note_amp, waveform=waveform
                        )
                        target_freq = int(note_freq(sweep_target))
                        if target_freq < 1:
                            target_freq = 1
                        elif target_freq > 1000:
                            target_freq = 1000
                        total_ticks = sound_ms * (_PWM_SAMPLE_RATE // 1000)
                        step_end = (target_freq << 32) // _PWM_SAMPLE_RATE
                        step_start = (freq << 32) // _PWM_SAMPLE_RATE
                        if total_ticks > 0 and step_start != step_end:
                            increment = (step_end - step_start) // total_ticks
                            pz_drive.pwm_set_sweep(step_end, increment)
                    else:
                        pa.set_frequency_live(
                            freq, amplitude=note_amp, waveform=waveform
                        )

                    time.sleep_ms(sound_ms)
                    prev_freq = freq

                    if gap_ms > 0:
                        pa.set_frequency_live(
                            freq, amplitude=0, waveform=waveform
                        )
                        time.sleep_ms(gap_ms)

            if not loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        pa.stop()


# ---------------------------------------------------------------------------
# Super Mario Bros — Overworld theme (Koji Kondo)
# Beats in eighth-note units. bpm=400.
# ---------------------------------------------------------------------------

SUPER_MARIO = [
    ("E5", 1), ("E5", 1), ("R", 1), ("E5", 1),
    ("R", 1), ("C5", 1), ("E5", 2),
    ("G5", 2), ("R", 2),
    ("G4", 2), ("R", 2),
    # phrase 2
    ("C5", 1), ("R", 1), ("R", 1), ("G4", 1),
    ("R", 2), ("E4", 2),
    ("R", 1), ("A4", 1), ("R", 1), ("B4", 1),
    ("R", 1), ("Bb4", 1), ("A4", 2),
    # phrase 3
    ("G4", 1.5), ("E5", 1.5), ("G5", 1),
    ("A5", 2), ("F5", 1), ("G5", 1),
    ("R", 1), ("E5", 1), ("R", 1), ("C5", 1),
    ("D5", 1), ("B4", 1), ("R", 2),
]

# ---------------------------------------------------------------------------
# Tetris — Korobeiniki (Russian folk melody)
# Beats in eighth-note units. bpm=300.
# ---------------------------------------------------------------------------

TETRIS = [
    ("E5", 2), ("B4", 1), ("C5", 1),
    ("D5", 2), ("C5", 1), ("B4", 1),
    ("A4", 2), ("A4", 1), ("C5", 1),
    ("E5", 2), ("D5", 1), ("C5", 1),
    ("B4", 3), ("C5", 1),
    ("D5", 2), ("E5", 2),
    ("C5", 2), ("A4", 2),
    ("A4", 2), ("R", 2),
    # phrase 2
    ("R", 1), ("D5", 2), ("F5", 1),
    ("A5", 2), ("G5", 1), ("F5", 1),
    ("E5", 3), ("C5", 1),
    ("E5", 2), ("D5", 1), ("C5", 1),
    ("B4", 2), ("B4", 1), ("C5", 1),
    ("D5", 2), ("E5", 2),
    ("C5", 2), ("A4", 2),
    ("A4", 2), ("R", 2),
]

# ---------------------------------------------------------------------------
# Happy Birthday
# Beats in quarter-note units. bpm=160.
# ---------------------------------------------------------------------------

HAPPY_BIRTHDAY = [
    ("C4", 0.75), ("C4", 0.25), ("D4", 1), ("C4", 1),
    ("F4", 1), ("E4", 2),
    ("C4", 0.75), ("C4", 0.25), ("D4", 1), ("C4", 1),
    ("G4", 1), ("F4", 2),
    ("C4", 0.75), ("C4", 0.25), ("C5", 1), ("A4", 1),
    ("F4", 1), ("E4", 1), ("D4", 1),
    ("Bb4", 0.75), ("Bb4", 0.25), ("A4", 1), ("F4", 1),
    ("G4", 1), ("F4", 2),
]


# ---------------------------------------------------------------------------
# Für Elise — Beethoven (opening theme, 3/8 time)
# Beats in eighth-note units. bpm=240 (eighth) ~ 80 bpm (quarter).
# ---------------------------------------------------------------------------

FUR_ELISE = [
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

# ---------------------------------------------------------------------------
# Nokia Tune — Gran Vals (Francisco Tarrega), 3/4 time
# Beats in eighth-note units. bpm=280.
# ---------------------------------------------------------------------------

NOKIA_TUNE = [
    ("E5", 1), ("D5", 1),
    ("F#4", 2), ("G#4", 2),
    ("C#5", 1), ("B4", 1),
    ("D4", 2), ("E4", 2),
    ("B4", 1), ("A4", 1),
    ("C#4", 2), ("E4", 2),
    ("A4", 4),
]

# ---------------------------------------------------------------------------
# Song registry: (data, bpm, gain)
# ---------------------------------------------------------------------------

SONGS = {
    "super_mario": (SUPER_MARIO, 400, 75),
    "tetris": (TETRIS, 300, 75),
    "happy_birthday": (HAPPY_BIRTHDAY, 160, 75),
    "fur_elise": (FUR_ELISE, 240, 50),
    "ode_to_joy": (ODE_TO_JOY, 120, 75),
    "imperial_march": (IMPERIAL_MARCH, 128, 100),
    "nokia": (NOKIA_TUNE, 360, 75),
}


def play_song(name, **kwargs):
    """Play a named song. Extra kwargs override defaults."""
    if name not in SONGS:
        raise ValueError(
            "unknown song: " + name + " (available: "
            + ", ".join(SONGS.keys()) + ")"
        )
    data, default_bpm, default_gain = SONGS[name]
    kwargs.setdefault("bpm", default_bpm)
    kwargs.setdefault("gain", default_gain)
    play(data, **kwargs)
