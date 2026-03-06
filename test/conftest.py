"""Root conftest -- path setup and mock injection for all tests."""

import os
import sys

_test_dir = os.path.dirname(__file__)
_mocks_dir = os.path.join(_test_dir, "mocks")
_repo_root = os.path.dirname(_test_dir)

# Mocks must be on path FIRST
sys.path.insert(0, _mocks_dir)

# Install MicroPython builtins BEFORE any app imports
import micropython_builtins  # noqa: E402, F401

# Add application code paths
sys.path.insert(1, os.path.join(_repo_root, "python", "frozen"))
sys.path.insert(2, os.path.join(_repo_root, "python"))

import pytest  # noqa: E402


@pytest.fixture(autouse=True)
def _reset_mocks():
    """Reset all mocks before each test."""
    import board_utils
    import pz_drive
    import sam

    pz_drive._reset()
    sam._reset()
    board_utils._reset()
    micropython_builtins._reset_all()
    yield
