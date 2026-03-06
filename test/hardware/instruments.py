"""Lightweight SCPI instrument clients for hardware E2E tests."""

import asyncio


class SCPIConnection:
    """Async TCP connection to a SCPI instrument."""

    def __init__(self, host: str, port: int, timeout: float = 10.0, drain_banner: bool = False):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._drain_banner = drain_banner
        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._lock = asyncio.Lock()

    async def connect(self):
        self._reader, self._writer = await asyncio.wait_for(
            asyncio.open_connection(self.host, self.port),
            timeout=self.timeout,
        )
        if self._drain_banner:
            await self._read_banner()

    async def _read_banner(self):
        buf = b""
        try:
            while b">>" not in buf:
                chunk = await asyncio.wait_for(self._reader.read(256), timeout=2.0)
                if not chunk:
                    break
                buf += chunk
        except asyncio.TimeoutError:
            pass

    async def disconnect(self):
        if self._writer:
            self._writer.close()
            try:
                await self._writer.wait_closed()
            except Exception:
                pass
        self._reader = None
        self._writer = None

    async def _ensure_connected(self):
        if self._writer is None or self._writer.is_closing():
            await self.connect()

    async def query(self, command: str) -> str:
        async with self._lock:
            await self._ensure_connected()
            self._writer.write(f"{command}\n".encode("ascii"))
            await self._writer.drain()
            try:
                response = await asyncio.wait_for(
                    self._reader.readline(),
                    timeout=self.timeout,
                )
                return response.decode("ascii").strip().lstrip("\x00")
            except asyncio.TimeoutError:
                await self.disconnect()
                raise

    async def write(self, command: str):
        async with self._lock:
            await self._ensure_connected()
            self._writer.write(f"{command}\n".encode("ascii"))
            await self._writer.drain()
            await asyncio.sleep(0.05)


class Oscilloscope:
    """Siglent SDS oscilloscope — measurement and configuration."""

    def __init__(self, host: str, port: int = 5025):
        self._conn = SCPIConnection(host, port)

    async def connect(self):
        await self._conn.connect()

    async def disconnect(self):
        await self._conn.disconnect()

    async def identify(self) -> str:
        return await self._conn.query("*IDN?")

    async def configure_channel(
        self,
        channel: str,
        vdiv: str,
        coupling: str = "D1M",
        trace: bool = True,
        probe: int = 10,
    ):
        await self._conn.write(f"{channel}:VDIV {vdiv}")
        await self._conn.write(f"{channel}:CPL {coupling}")
        await self._conn.write(f"{channel}:TRA {'ON' if trace else 'OFF'}")
        await self._conn.write(f"{channel}:ATTN {probe}")

    async def configure_timebase(self, timebase: str):
        await self._conn.write(f"TDIV {timebase}")

    async def configure_trigger(self, source: str, level: str, slope: str = "POS"):
        await self._conn.write(f"{source}:TRLV {level}")
        await self._conn.write(f"{source}:TRSL {slope}")
        await self._conn.write("TRSE EDGE,SR," + source + ",HT,OFF")

    async def run(self):
        await self._conn.write("ARM")

    async def stop(self):
        await self._conn.write("STOP")

    async def measure(self, channel: str, parameter: str) -> str:
        result = await self._conn.query(f"{channel}:PAVA? {parameter}")
        parts = result.split(",")
        if len(parts) >= 2:
            val = parts[1].rstrip(
                "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ%"
            )
            return val
        return result

    async def measure_float(self, channel: str, parameter: str) -> float | None:
        val = await self.measure(channel, parameter)
        if "****" in val or not val:
            return None
        return float(val)


class PowerSupply:
    """Siglent SPD power supply — output control and measurement."""

    def __init__(self, host: str, port: int = 5025):
        self._conn = SCPIConnection(host, port)

    async def connect(self):
        await self._conn.connect()

    async def disconnect(self):
        await self._conn.disconnect()

    async def identify(self) -> str:
        return await self._conn.query("*IDN?")

    async def set_output(self, channel: str, state: bool):
        await self._conn.write(f"OUTPut {channel},{'ON' if state else 'OFF'}")

    async def set_voltage(self, channel: str, voltage: float):
        await self._conn.write(f"{channel}:VOLTage {voltage:.3f}")

    async def set_current(self, channel: str, current: float):
        await self._conn.write(f"{channel}:CURRent {current:.3f}")

    async def measure_voltage(self, channel: str) -> float:
        result = await self._conn.query(f"MEASure:VOLTage? {channel}")
        return float(result)

    async def measure_current(self, channel: str) -> float:
        result = await self._conn.query(f"MEASure:CURRent? {channel}")
        return float(result)

    async def measure_power(self, channel: str) -> float:
        result = await self._conn.query(f"MEASure:POWEr? {channel}")
        return float(result)


class Multimeter:
    """Siglent SDM multimeter — precision current measurement."""

    def __init__(self, host: str, port: int = 5024):
        self._conn = SCPIConnection(host, port, drain_banner=True)

    async def connect(self):
        await self._conn.connect()

    async def disconnect(self):
        await self._conn.disconnect()

    async def identify(self) -> str:
        return await self._conn.query("*IDN?")

    async def configure_dc_current(self, range: str = "6"):
        await self._conn.write(f"CONFigure:CURRent:DC {range}")

    async def configure_dc_voltage(self, range: str = "AUTO"):
        if range.upper() == "AUTO":
            await self._conn.write("CONFigure:VOLTage:DC")
            await self._conn.write("VOLTage:DC:RANGe:AUTO ON")
        else:
            await self._conn.write(f"CONFigure:VOLTage:DC {range}")

    async def read(self) -> float:
        result = await self._conn.query("READ?")
        return float(result)

    async def measure_dc_current(self, range: str = "6") -> float:
        result = await self._conn.query(f"MEASure:CURRent:DC? {range}")
        return float(result)
