"""Synchronous WebSocket wrapper for board commands."""

import json

from websocket import WebSocket


class BoardClient:
    """Thin sync wrapper around a WebSocket connection to the board."""

    def __init__(self, url: str):
        self._url = url
        self._ws = WebSocket()

    def connect(self):
        self._ws.connect(self._url)
        self._ws.recv()  # initial status message

    def send_cmd(self, cmd: str, **kwargs) -> dict:
        """Send a command and return the parsed JSON response."""
        payload = {"cmd": cmd, **kwargs}
        self._ws.send(json.dumps(payload))
        return json.loads(self._ws.recv())

    def stop(self) -> dict:
        status = self.send_cmd("stop")
        assert status.get("running") is False, f"Board did not stop: {status}"
        return status

    def start(self, gain: int = 100) -> dict:
        status = self.send_cmd("start", gain=gain)
        assert status.get("running") is True, f"Board did not start: {status}"
        assert status.get("gain") == gain, f"Board did not accept gain={gain}: {status}"
        return status

    def set_frequency_analog(
        self,
        hz: int,
        amplitude: int = 100,
        waveform: str = "sine",
        fullwave: bool = False,
    ) -> dict:
        status = self.send_cmd(
            "set_frequency_analog",
            hz=hz,
            amplitude=amplitude,
            waveform=waveform,
            fullwave=fullwave,
        )
        assert status.get("frequency") == hz, (
            f"Board did not accept frequency={hz}: {status}"
        )
        assert status.get("fullwave") == fullwave, (
            f"Board did not accept fullwave={fullwave}: {status}"
        )
        return status

    def set_pin(self, pin: int, value: int) -> dict:
        return self.send_cmd("set_pin", pin=pin, value=value)

    def set_all(self, value: int) -> dict:
        return self.send_cmd("set_all", value=value)

    def close(self):
        self._ws.close()
