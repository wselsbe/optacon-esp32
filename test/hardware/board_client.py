"""WebSocket wrapper for board commands — removes json.dumps/recv boilerplate."""

import json


class BoardClient:
    """Thin async wrapper around a websockets connection to the board."""

    def __init__(self, ws):
        self._ws = ws

    async def send_cmd(self, cmd: str, **kwargs) -> dict:
        """Send a command and return the parsed JSON response."""
        payload = {"cmd": cmd, **kwargs}
        await self._ws.send(json.dumps(payload))
        return json.loads(await self._ws.recv())

    async def stop(self) -> dict:
        return await self.send_cmd("stop")

    async def start(self, gain: int = 100) -> dict:
        return await self.send_cmd("start", gain=gain)

    async def set_frequency_analog(
        self,
        hz: int,
        amplitude: int = 100,
        waveform: str = "sine",
        fullwave: bool = False,
    ) -> dict:
        return await self.send_cmd(
            "set_frequency_analog",
            hz=hz,
            amplitude=amplitude,
            waveform=waveform,
            fullwave=fullwave,
        )

    async def set_pin(self, pin: int, value: int) -> dict:
        return await self.send_cmd("set_pin", pin=pin, value=value)

    async def set_all(self, value: int) -> dict:
        return await self.send_cmd("set_all", value=value)

    async def close(self):
        await self._ws.close()
