"""Lightweight SCPI instrument clients for hardware E2E tests."""

import contextlib
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
            except TimeoutError:
                break

    def disconnect(self):
        if self._sock:
            with contextlib.suppress(Exception):
                self._sock.close()
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

    def query(self, command: str, retries: int = 1) -> str:
        with self._lock:
            for attempt in range(1 + retries):
                self._ensure_connected()
                try:
                    self._sock.sendall(f"{command}\n".encode("ascii"))
                    response = self._readline()
                    return response.decode("ascii").strip().lstrip("\x00")
                except (ConnectionResetError, ConnectionError, TimeoutError):
                    self.disconnect()
                    if attempt >= retries:
                        raise
                    time.sleep(0.5)

    def read_binary(self, command: str) -> bytes:
        """Send command and read binary response (non-blocking drain)."""
        with self._lock:
            self._ensure_connected()
            self._sock.setblocking(False)
            try:
                self._sock.sendall(f"{command}\n".encode("ascii"))
                time.sleep(1)  # let scope prepare the response
                data = b""
                while True:
                    try:
                        time.sleep(0.01)
                        chunk = self._sock.recv(8000)
                        data += chunk
                    except BlockingIOError:
                        break
                return data
            finally:
                self._sock.settimeout(self.timeout)

    def write(self, command: str, retries: int = 1):
        with self._lock:
            for attempt in range(1 + retries):
                self._ensure_connected()
                try:
                    self._sock.sendall(f"{command}\n".encode("ascii"))
                    time.sleep(0.05)
                    return
                except (ConnectionResetError, ConnectionError):
                    self.disconnect()
                    if attempt >= retries:
                        raise
                    time.sleep(0.5)


class Oscilloscope:
    """Siglent SDS oscilloscope — measurement and configuration.

    Tracks state to skip redundant SCPI writes and uses *OPC? to wait
    for operations to complete before measuring.
    """

    def __init__(self, host: str, port: int = 5025):
        self._conn = SCPIConnection(host, port)
        self._state: dict[str, str] = {}

    def connect(self):
        self._conn.connect()
        self._state.clear()

    def disconnect(self):
        self._conn.disconnect()
        self._state.clear()

    def identify(self) -> str:
        return self._conn.query("*IDN?")

    def _write_if_changed(self, key: str, command: str):
        """Only send command if state has changed."""
        if self._state.get(key) != command:
            self._conn.write(command)
            self._state[key] = command

    def configure_channel(
        self,
        channel: str,
        vdiv: str,
        coupling: str = "D1M",
        trace: bool = True,
        probe: int = 10,
    ):
        # ATTN must be set before VDIV — Siglent rescales VDIV when ATTN changes
        attn_cmd = f"{channel}:ATTN {probe}"
        if self._state.get(f"{channel}:ATTN") != attn_cmd:
            self._conn.write(attn_cmd)
            self._state[f"{channel}:ATTN"] = attn_cmd
            # Invalidate cached VDIV — scope rescaled it when ATTN changed
            self._state.pop(f"{channel}:VDIV", None)
        self._write_if_changed(f"{channel}:VDIV", f"{channel}:VDIV {vdiv}")
        self._write_if_changed(f"{channel}:CPL", f"{channel}:CPL {coupling}")
        tra = "ON" if trace else "OFF"
        self._write_if_changed(f"{channel}:TRA", f"{channel}:TRA {tra}")

    def configure_timebase(self, timebase: str):
        self._write_if_changed("TDIV", f"TDIV {timebase}")

    def configure_trigger(self, source: str, level: str, slope: str = "POS"):
        self._write_if_changed("TRLV", f"{source}:TRLV {level}")
        self._write_if_changed("TRSL", f"{source}:TRSL {slope}")
        trse = "TRSE EDGE,SR," + source + ",HT,OFF"
        self._write_if_changed("TRSE", trse)
        # Use auto trigger mode so scope acquires even without trigger event
        self._write_if_changed("TRMD", "TRMD AUTO")

    def wait_ready(self):
        """Wait for all pending operations to complete."""
        self._conn.query("*OPC?")

    def run(self):
        # Always send TRMD AUTO (bypass cache) to restart acquisition
        self._conn.write("TRMD AUTO")
        self._state["TRMD"] = "TRMD AUTO"

    def stop(self):
        self._conn.write("STOP")

    _NUMERIC_RE = re.compile(r"[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?")

    def measure(self, channel: str, parameter: str) -> str:
        result = self._conn.query(f"{channel}:PAVA? {parameter}")
        parts = result.split(",")
        if len(parts) >= 2:
            return parts[1]
        return result

    def screenshot(self) -> bytes:
        """Capture screen as BMP image."""
        return self._conn.read_binary("SCDP")

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

    def is_output_on(self, channel: str, threshold: float = 1.0) -> bool:
        """Check if output is on by measuring voltage (> threshold means on)."""
        try:
            voltage = self.measure_voltage(channel)
            return voltage > threshold
        except Exception:
            return False

    def ensure_output_on(self, channel: str, voltage: float, current_limit: float):
        """Set voltage/current and enable output if not already on."""
        if current_limit > 1.0:
            raise ValueError(f"Current limit {current_limit}A exceeds 1A safety cap")
        if self.is_output_on(channel):
            return
        self.set_voltage(channel, voltage)
        self.set_current(channel, current_limit)
        self.set_output(channel, True)


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
