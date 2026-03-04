"""Sheet music player for piezo actuators via PWM.

Plays sequences of notes defined as (note_name, duration_in_beats) tuples.
Uses the analog PWM path through PzActuator.

Usage:
    from music import play, CLAIR_DE_LUNE

    play(CLAIR_DE_LUNE)                    # play with defaults
    play(CLAIR_DE_LUNE, bpm=54, gain=50)   # slower, quieter

    # Custom song: list of (note, beats) tuples
    my_song = [
        ("C4", 1), ("E4", 1), ("G4", 1), ("C5", 2),
        ("R", 1),  # rest
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

def play(song, bpm=72, gain=100, waveform="sine", amplitude=100,
         staccato_ratio=0.9, loop=False):
    """Play a song: list of (note, beats) tuples.

    Args:
        song: list of (note_name, duration_in_beats) — use "R" or None for rests
        bpm: tempo in beats per minute (default 72)
        gain: DRV2665 gain — 25, 50, 75, or 100 Vpp
        waveform: 'sine', 'triangle', or 'square'
        amplitude: 0-100 PWM amplitude percentage
        staccato_ratio: fraction of note duration to sound (0.0-1.0);
                        remainder is silence between notes (default 0.9)
        loop: if True, repeat until KeyboardInterrupt
    """
    from pz_drive_py import PzActuator

    pa = PzActuator()
    beat_ms = int(60_000 / bpm)
    started = False

    try:
        while True:
            for note, beats in song:
                dur_ms = int(beat_ms * beats)
                sound_ms = int(dur_ms * staccato_ratio)
                gap_ms = dur_ms - sound_ms

                is_rest = note is None or note == "R"

                if is_rest:
                    # Rest: silence for full duration
                    if started:
                        pa.stop()
                        started = False
                    time.sleep_ms(dur_ms)
                else:
                    freq = note_freq(note)
                    # Clamp to hardware range
                    if freq < 1:
                        freq = 1
                    elif freq > 500:
                        freq = 500
                    pa.set_frequency_analog(
                        int(freq), amplitude=amplitude, waveform=waveform
                    )
                    if not started:
                        pa.start(gain=gain)
                        started = True
                    time.sleep_ms(sound_ms)
                    # Brief gap between notes for articulation
                    if gap_ms > 0:
                        pa.stop()
                        started = False
                        time.sleep_ms(gap_ms)

            if not loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if started:
            pa.stop()


# ---------------------------------------------------------------------------
# Clair de Lune — Debussy (opening measures, simplified melody)
#
# The piece is in Db major, 9/8 time. Below is a reduction of the
# right-hand melody from the first ~16 measures at the iconic opening.
# Beats are in eighth-note units (1 beat = one eighth note).
# At bpm=200 (eighth = 200 bpm) this gives a gentle tempo of ~66 bpm
# in dotted-quarter terms, close to the typical performance tempo.
# ---------------------------------------------------------------------------

CLAIR_DE_LUNE = [
    # Measure 1-2: soft opening motif (Db major)
    ("R", 3),
    ("R", 3),
    ("R", 3),
    # Measure 3: melody enters
    ("Bb3", 2), ("Bb3", 1),
    ("Bb3", 2), ("Ab3", 1),
    ("Bb3", 2), ("Db4", 1),
    # Measure 4
    ("Gb4", 3),
    ("Gb4", 3),
    ("F4", 3),
    # Measure 5
    ("Eb4", 2), ("Eb4", 1),
    ("Eb4", 2), ("Db4", 1),
    ("Eb4", 2), ("Gb4", 1),
    # Measure 6
    ("Bb4", 3),
    ("Bb4", 3),
    ("Ab4", 3),
    # Measure 7
    ("Gb4", 2), ("Gb4", 1),
    ("Gb4", 2), ("F4", 1),
    ("Gb4", 2), ("Ab4", 1),
    # Measure 8
    ("Bb4", 3),
    ("Db5", 3),
    ("Ab4", 3),
    # Measure 9
    ("Gb4", 2), ("Gb4", 1),
    ("Gb4", 2), ("Ab4", 1),
    ("Gb4", 2), ("F4", 1),
    # Measure 10
    ("Eb4", 3),
    ("Db4", 3),
    ("Eb4", 3),
    # Measure 11
    ("Bb3", 2), ("Bb3", 1),
    ("Bb3", 2), ("Ab3", 1),
    ("Bb3", 2), ("Db4", 1),
    # Measure 12
    ("Gb4", 3),
    ("Gb4", 3),
    ("F4", 3),
    # Measure 13
    ("Eb4", 2), ("Eb4", 1),
    ("Eb4", 2), ("Db4", 1),
    ("Eb4", 2), ("Gb4", 1),
    # Measure 14
    ("Bb4", 4.5),
    ("Ab4", 4.5),
    # Measure 15
    ("Gb4", 3),
    ("F4", 3),
    ("Eb4", 3),
    # Measure 16
    ("Db4", 9),
]

# Suggested playback: play(CLAIR_DE_LUNE, bpm=200, gain=50, waveform='sine')
