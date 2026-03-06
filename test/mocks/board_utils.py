"""Mock of the board_utils C module for testing under CPython."""

from unittest.mock import MagicMock

enter_bootloader = MagicMock()


def _reset():
    """Reset all mocks and restore defaults."""
    enter_bootloader.reset_mock()
