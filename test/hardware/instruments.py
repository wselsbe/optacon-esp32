"""Lightweight SCPI instrument clients for hardware E2E tests."""

import re
import socket
import threading
import time


class SCPIConnection:
    """Synchronous TCP connection to a SCPI instrument."""

    def __init__(self, host: str, port: int, timeout: float = 10.0, drain_banner: bool = False):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._drain_banner = drain_banner
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._buf = b""

    def connect(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(self.timeout)
        self._sock.connect((self.host, self.port))
        self._buf = b""
        if self._drain_banner:
            self._read_banner()

    def _read_banner(self):
        buf = b""
        deadline = time.monotonic() + 2.0
        while b">>" not in buf and time.monotonic() < deadline:
            try:
                chunk = self._sock.recv(256)
                if not chunk:
                    break
                buf += chunk
            except socket.timeout:
                break

    def disconnect(self):
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None
        self._buf = b""

    def _ensure_connected(self):
        if self._sock is None:
            self.connect()

    def _readline(self) -> bytes:
        """Read until newline from socket, using internal buffer."""
        while b"\n" not in self._buf:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionError("Socket closed")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return line

    def query(self, command: str) -> str:
        with self._lock:
            self._ensure_connected()
            self._sock.sendall(f"{command}\n".encode("ascii"))
            try:
                response = self._readline()
                return response.decode("ascii").strip().lstrip("\x00")
            except socket.timeout:
                self.disconnect()
                raise

    def write(self, command: str):
        with self._lock:
            self._ensure_connected()
            self._sock.sendall(f"{command}\n".encode("ascii"))
            time.sleep(0.05)


class Oscilloscope:
    """Siglent SDS oscilloscope — measurement and configuration."""

    def __init__(self, host: str, port: int = 5025):
        self._conn = SCPIConnection(host, port)

    def connect(self):
        self._conn.connect()

    def disconnect(self):
        self._conn.disconnect()

    def identify(self) -> str:
        return self._conn.query("*IDN?")

    def configure_channel(
        self,
        channel: str,
        vdiv: str,
        coupling: str = "D1M",
        trace: bool = True,
        probe: int = 10,
    ):
        self._conn.write(f"{channel}:VDIV {vdiv}")
        self._conn.write(f"{channel}:CPL {coupling}")
        self._conn.write(f"{channel}:TRA {'ON' if trace else 'OFF'}")
        self._conn.write(f"{channel}:ATTN {probe}")

    def configure_timebase(self, timebase: str):
        self._conn.write(f"TDIV {timebase}")

    def configure_trigger(self, source: str, level: str, slope: str = "POS"):
        self._conn.write(f"{source}:TRLV {level}")
        self._conn.write(f"{source}:TRSL {slope}")
        self._conn.write("TRSE EDGE,SR," + source + ",HT,OFF")

    def run(self):
        self._conn.write("ARM")

    def stop(self):
        self._conn.write("STOP")

    _NUMERIC_RE = re.compile(r"[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?")

    def measure(self, channel: str, parameter: str) -> str:
        result = self._conn.query(f"{channel}:PAVA? {parameter}")
        parts = result.split(",")
        if len(parts) >= 2:
            return parts[1]
        return result

    def measure_float(self, channel: str, parameter: str) -> float | None:
        val = self.measure(channel, parameter)
        if "****" in val or not val:
            return None
        m = self._NUMERIC_RE.search(val)
        if m is None:
            return None
        return float(m.group())


class PowerSupply:
    """Siglent SPD power supply — output control and measurement."""

    def __init__(self, host: str, port: int = 5025):
        self._conn = SCPIConnection(host, port)

    def connect(self):
        self._conn.connect()

    def disconnect(self):
        self._conn.disconnect()

    def identify(self) -> str:
        return self._conn.query("*IDN?")

    def set_output(self, channel: str, state: bool):
        self._conn.write(f"OUTPut {channel},{'ON' if state else 'OFF'}")

    def set_voltage(self, channel: str, voltage: float):
        self._conn.write(f"{channel}:VOLTage {voltage:.3f}")

    def set_current(self, channel: str, current: float):
        self._conn.write(f"{channel}:CURRent {current:.3f}")

    def measure_voltage(self, channel: str) -> float:
        result = self._conn.query(f"MEASure:VOLTage? {channel}")
        return float(result)

    def measure_current(self, channel: str) -> float:
        result = self._conn.query(f"MEASure:CURRent? {channel}")
        return float(result)

    def measure_power(self, channel: str) -> float:
        result = self._conn.query(f"MEASure:POWEr? {channel}")
        return float(result)


class Multimeter:
    """Siglent SDM multimeter — precision current measurement."""

    def __init__(self, host: str, port: int = 5024):
        self._conn = SCPIConnection(host, port, drain_banner=True)

    def connect(self):
        self._conn.connect()

    def disconnect(self):
        self._conn.disconnect()

    def identify(self) -> str:
        return self._conn.query("*IDN?")

    def configure_dc_current(self, range: str = "6"):
        self._conn.write(f"CONFigure:CURRent:DC {range}")

    def configure_dc_voltage(self, range: str = "AUTO"):
        if range.upper() == "AUTO":
            self._conn.write("CONFigure:VOLTage:DC")
            self._conn.write("VOLTage:DC:RANGe:AUTO ON")
        else:
            self._conn.write(f"CONFigure:VOLTage:DC {range}")

    def read(self) -> float:
        result = self._conn.query("READ?")
        return float(result)

    def measure_dc_current(self, range: str = "6") -> float:
        result = self._conn.query(f"MEASure:CURRent:DC? {range}")
        return float(result)
