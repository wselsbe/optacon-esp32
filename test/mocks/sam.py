"""Mock of the sam C module for testing under CPython."""

from unittest.mock import MagicMock

say = MagicMock()
render = MagicMock(return_value=bytearray(100))


def _reset():
    """Reset all mocks and restore defaults."""
    say.reset_mock()
    render.reset_mock(return_value=True)
    render.return_value = bytearray(100)
