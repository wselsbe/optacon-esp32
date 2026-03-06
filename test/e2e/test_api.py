"""Smoke test -- verify web_server.py creates app with mock deps."""

import pytest
from unittest.mock import MagicMock


def test_create_app_with_mock_deps():
    """web_server.create_app() returns a Microdot app with mock deps."""
    import web_server

    deps = MagicMock()
    deps.pa.get_status.return_value = {
        "running": False,
        "mode": None,
        "frequency": 0,
        "gain": 100,
        "fullwave": False,
        "waveform": "sine",
        "polarity": False,
        "pins": [0] * 20,
    }
    deps.wifi.get_status.return_value = {
        "mode": "ap",
        "ssid": "test",
        "ip": "192.168.4.1",
        "hostname": "test.local",
    }

    app = web_server.create_app(deps)
    assert app is not None
    # Verify it's a Microdot instance
    assert hasattr(app, "start_server")
