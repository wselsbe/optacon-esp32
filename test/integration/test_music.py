"""Integration tests for the music module."""

import time

import pytest
from music import NOTES, SONGS, note_freq, play_song


@pytest.fixture(autouse=True)
def fast_sleep(monkeypatch):
    monkeypatch.setattr(time, "sleep_ms", lambda ms: None)


class TestNoteFreq:
    def test_a4_is_440(self):

        assert note_freq("A4") == 440.0

    def test_c4_approx_261(self):

        assert abs(note_freq("C4") - 261.63) < 0.01

    def test_unknown_raises(self):

        with pytest.raises(ValueError, match="unknown note"):
            note_freq("X9")

    def test_enharmonic_aliases(self):

        assert note_freq("C#4") == note_freq("Db4")
        assert note_freq("F#4") == note_freq("Gb4")
        assert note_freq("A#4") == note_freq("Bb4")


class TestNotesDict:
    def test_all_octaves_present(self):

        for octave in range(2, 7):
            for name in ("C", "Db", "D", "Eb", "E", "F", "Gb", "G", "Ab", "A", "Bb", "B"):
                key = name + str(octave)
                assert key in NOTES, f"Missing {key}"

    def test_12_notes_per_octave(self):

        base_names = ("C", "Db", "D", "Eb", "E", "F", "Gb", "G", "Ab", "A", "Bb", "B")
        for octave in range(2, 7):
            count = sum(1 for n in base_names if n + str(octave) in NOTES)
            assert count == 12


class TestSongsDict:
    def test_has_7_entries(self):

        assert len(SONGS) == 7

    def test_each_entry_is_tuple(self):

        for name, entry in SONGS.items():
            assert isinstance(entry, tuple), f"{name} is not a tuple"
            assert len(entry) == 3, f"{name} has {len(entry)} elements"
            assert isinstance(entry[0], list), f"{name}[0] is not a list"
            assert isinstance(entry[1], int), f"{name}[1] (bpm) is not int"
            assert isinstance(entry[2], int), f"{name}[2] (gain) is not int"


class TestPlaySong:
    def test_play_super_mario(self):

        play_song("super_mario")

    def test_nonexistent_raises(self):

        with pytest.raises(ValueError, match="unknown song"):
            play_song("nonexistent")

    def test_bpm_override(self):

        # Should not raise
        play_song("tetris", bpm=200)
