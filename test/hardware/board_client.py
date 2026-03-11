"""Board client implementations: WebSocket API and REPL (mpremote)."""

import json
import subprocess


class BoardClient:
    """Base board client with shared assertions."""

    # DRV2665 full-scale is 1.8Vpp differential; amplitude=55 drives ~2.5Vpp on
    # IN+, slightly overdriving the input. There is unwanted ~0.8Vpp coupling on
    # IN- (cause unknown) which reduces the effective differential swing. To be
    # reviewed in a future hardware revision.
    DEFAULT_AMPLITUDE = 55

    def connect(self):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError

    def _exec(self, code: str) -> str:
        """Execute Python code on the board, return stdout."""
        raise NotImplementedError

    def stop(self) -> dict:
        self._exec("pa.stop()")
        status = self.get_status()
        assert status.get("running") is False, f"Board did not stop: {status}"
        return status

    def start(self, gain: int = 100) -> dict:
        self._exec(f"pa.start(gain={gain})")
        status = self.get_status()
        assert status.get("running") is True, f"Board did not start: {status}"
        assert status.get("gain") == gain, f"Board did not accept gain={gain}: {status}"
        return status

    def set_frequency_analog(
        self,
        hz: int,
        amplitude: int = DEFAULT_AMPLITUDE,
        waveform: str = "sine",
        fullwave: bool = False,
    ) -> dict:
        self._exec(
            f"pa.set_frequency_analog({hz}, amplitude={amplitude}, "
            f"fullwave={fullwave}, waveform='{waveform}')"
        )
        status = self.get_status()
        assert status.get("frequency") == hz, (
            f"Board did not accept frequency={hz}: {status}"
        )
        assert status.get("fullwave") == fullwave, (
            f"Board did not accept fullwave={fullwave}: {status}"
        )
        return status

    def set_pin(self, pin: int, value: int) -> dict:
        self._exec(f"pa.shift_register.set_pin({pin}, {value})")
        return self.get_status()

    def set_all(self, value: int) -> dict:
        self._exec(f"pa.shift_register.set_all({value})")
        return self.get_status()

    def get_status(self) -> dict:
        raise NotImplementedError


class WSBoardClient(BoardClient):
    """Board client using WebSocket API."""

    def __init__(self, url: str):
        from websocket import WebSocket

        self._url = url
        self._ws = WebSocket()

    def connect(self):
        self._ws.connect(self._url)
        self._ws.recv()  # initial status message

    def close(self):
        self._ws.close()

    def _send_cmd(self, cmd: str, **kwargs) -> dict:
        payload = {"cmd": cmd, **kwargs}
        self._ws.send(json.dumps(payload))
        return json.loads(self._ws.recv())

    def _exec(self, code: str) -> str:
        return str(self._send_cmd("get_status"))

    def get_status(self) -> dict:
        return self._send_cmd("get_status")

    # Override base methods to use WS commands directly (more efficient)
    def stop(self) -> dict:
        status = self._send_cmd("stop")
        assert status.get("running") is False, f"Board did not stop: {status}"
        return status

    def start(self, gain: int = 100) -> dict:
        status = self._send_cmd("start", gain=gain)
        assert status.get("running") is True, f"Board did not start: {status}"
        assert status.get("gain") == gain, f"Board did not accept gain={gain}: {status}"
        return status

    def set_frequency_analog(
        self,
        hz: int,
        amplitude: int = BoardClient.DEFAULT_AMPLITUDE,
        waveform: str = "sine",
        fullwave: bool = False,
    ) -> dict:
        status = self._send_cmd(
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
        return self._send_cmd("set_pin", pin=pin, value=value)

    def set_all(self, value: int) -> dict:
        return self._send_cmd("set_all", value=value)


class REPLBoardClient(BoardClient):
    """Board client using mpremote REPL (serial connection)."""

    def __init__(self, port: str | None = None):
        self._port = port
        self._base_cmd = ["mpremote"]
        if port:
            self._base_cmd += ["connect", port]

    def connect(self):
        # Verify board is reachable
        result = self._exec("import sys; print(sys.version)")
        assert result.strip(), "Could not connect to board via mpremote"

    def close(self):
        pass  # mpremote uses stateless per-call connections

    def _exec(self, code: str) -> str:
        result = subprocess.run(
            self._base_cmd + ["exec", code],
            capture_output=True,
            text=True,
            timeout=15,
        )
        if result.returncode != 0:
            raise RuntimeError(f"mpremote exec failed: {result.stderr}")
        return result.stdout

    def get_status(self) -> dict:
        output = self._exec("import json; print(json.dumps(pa.get_status()))")
        return json.loads(output.strip())
