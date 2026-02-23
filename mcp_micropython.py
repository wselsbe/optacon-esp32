"""MCP server for MicroPython boards via mpremote serial transport.

TODO: Investigate mpremote over network (WebREPL / TCP transport).
      - How to enable networking on the ESP32-S3 (WiFi STA/AP config)
      - How to set up WebREPL on the board (webrepl_setup, boot.py config)
      - Whether mpremote supports network targets natively or needs a different transport
      - Latency / reliability tradeoffs vs USB-CDC serial
"""

import logging
import os
import stat
import sys
import time
from contextlib import contextmanager

import serial.tools.list_ports
from fastmcp import FastMCP
from mpremote.transport_serial import SerialTransport

# Logging to stderr (stdout reserved for MCP JSON-RPC)
logging.basicConfig(stream=sys.stderr, level=logging.INFO)
log = logging.getLogger("mcp-micropython")

mcp = FastMCP(
    "micropython",
    instructions="Interact with a MicroPython board over serial. "
    "Use exec to run code, list_files/read_file/write_file for filesystem ops, "
    "device_info for board details, soft_reset to reset the board.",
)

# --- Configuration from environment ---

MPY_PORT = os.environ.get("MPY_PORT")
MPY_VID = os.environ.get("MPY_VID")  # e.g. "0x303A" for Espressif
MPY_BAUD = int(os.environ.get("MPY_BAUD", "115200"))
MPY_EXEC_TIMEOUT = int(os.environ.get("MPY_EXEC_TIMEOUT", "5"))

# --- Persistent connection ---

_persistent: SerialTransport | None = None


def find_device() -> str | None:
    """Find MicroPython device: explicit port > VID scan > None."""
    if MPY_PORT:
        return MPY_PORT
    if MPY_VID:
        vid = int(MPY_VID, 16)
        for p in serial.tools.list_ports.comports():
            if p.vid == vid:
                log.info("Auto-detected device on %s (VID=0x%04X)", p.device, vid)
                return p.device
    return None


def _close_persistent():
    """Close the persistent connection if open."""
    global _persistent
    if _persistent is not None:
        try:
            _persistent.exit_raw_repl()
        except Exception:
            pass
        try:
            _persistent.close()
        except Exception:
            pass
        _persistent = None


def _get_persistent() -> SerialTransport:
    """Get or create the persistent connection."""
    global _persistent
    if _persistent is not None:
        # Verify it's still alive
        try:
            _persistent.serial.inWaiting()
            return _persistent
        except Exception:
            log.info("Persistent connection stale, reconnecting")
            _close_persistent()

    port = find_device()
    if not port:
        raise RuntimeError(
            "No MicroPython device found. "
            "Set MPY_PORT or MPY_VID environment variable."
        )
    t = SerialTransport(port, baudrate=MPY_BAUD)
    t.enter_raw_repl(soft_reset=False)
    _persistent = t
    return t


@contextmanager
def connect(soft_reset=False, keep_open=False):
    """Connect to device, yield transport.

    If keep_open=True, reuse a persistent connection (port stays open).
    If keep_open=False, open and close the port for this call only.
    """
    if keep_open:
        t = _get_persistent()
        yield t
        # Don't close — keep for next call
    else:
        # Close any persistent connection first to free the port
        _close_persistent()
        port = find_device()
        if not port:
            raise RuntimeError(
                "No MicroPython device found. "
                "Set MPY_PORT or MPY_VID environment variable."
            )
        t = SerialTransport(port, baudrate=MPY_BAUD)
        try:
            t.enter_raw_repl(soft_reset=soft_reset)
            yield t
        finally:
            try:
                t.exit_raw_repl()
            except Exception:
                pass
            t.close()


# --- MCP Tools ---


@mcp.tool()
def exec(code: str, timeout: int = 0, keep_open: bool = False) -> str:
    """Execute MicroPython code on the board and return stdout.

    Args:
        code: Python code to execute on the device.
        timeout: Timeout in seconds waiting for output (0 = use MPY_EXEC_TIMEOUT env default).
        keep_open: If true, keep the serial port open between calls for faster repeated access.
    """
    t_out = timeout if timeout > 0 else MPY_EXEC_TIMEOUT
    with connect(keep_open=keep_open) as t:
        t.exec_raw_no_follow(code)
        ret, ret_err = t.follow(timeout=t_out)
        if ret_err:
            from mpremote.transport import TransportExecError
            raise TransportExecError(ret, ret_err.decode())
        return ret.decode(errors="replace")


@mcp.tool()
def enter_bootloader() -> str:
    """Reset the board into USB bootloader mode for flashing.

    Sends the enter_bootloader() command which disconnects USB.
    The serial exception is expected and swallowed.
    After this, use esptool/flash.cmd to flash new firmware.
    """
    _close_persistent()
    port = find_device()
    if not port:
        return "No MicroPython device found."

    try:
        t = SerialTransport(port, baudrate=MPY_BAUD)
        t.enter_raw_repl(soft_reset=False)
        try:
            t.exec_raw_no_follow("import pz_actuator; pz_actuator.enter_bootloader()")
            # The board disconnects USB — follow() will fail
            time.sleep(0.5)
        except Exception:
            pass
        try:
            t.close()
        except Exception:
            pass
    except Exception as e:
        # Any serial error during this process is expected
        log.info("enter_bootloader serial exception (expected): %s", e)

    return "Board entering bootloader mode. USB disconnected. Ready for flashing."


@mcp.tool()
def disconnect() -> str:
    """Close the persistent serial connection (if open)."""
    _close_persistent()
    return "Disconnected."


@mcp.tool()
def soft_reset() -> str:
    """Soft-reset the MicroPython board (equivalent to Ctrl-D)."""
    _close_persistent()
    with connect(soft_reset=True) as t:
        result = t.exec("print('reset ok')")
        return result.decode(errors="replace")


@mcp.tool()
def list_files(path: str = "/", keep_open: bool = False) -> str:
    """List files and directories on the device filesystem.

    Args:
        path: Directory path to list (default: root "/").
        keep_open: If true, keep the serial port open between calls.
    """
    with connect(keep_open=keep_open) as t:
        entries = t.fs_listdir(path)
        lines = []
        for entry in entries:
            # entry is (name, st_mode, st_ino, st_size)
            name = entry[0]
            mode = entry[1]
            size = entry[3]
            kind = "dir" if stat.S_ISDIR(mode) else "file"
            lines.append(f"{kind:4s}  {size:>8d}  {name}")
        return "\n".join(lines) if lines else "(empty)"


@mcp.tool()
def read_file(path: str, keep_open: bool = False) -> str:
    """Read a file from the device filesystem.

    Args:
        path: File path on the device (e.g. "/main.py").
        keep_open: If true, keep the serial port open between calls.
    """
    with connect(keep_open=keep_open) as t:
        data = t.fs_readfile(path)
        return data.decode(errors="replace")


@mcp.tool()
def write_file(path: str, content: str, keep_open: bool = False) -> str:
    """Write content to a file on the device filesystem.

    Args:
        path: Destination file path on the device (e.g. "/main.py").
        content: File content to write.
        keep_open: If true, keep the serial port open between calls.
    """
    with connect(keep_open=keep_open) as t:
        t.fs_writefile(path, content.encode())
        return f"Wrote {len(content)} bytes to {path}"


@mcp.tool()
def device_info(keep_open: bool = False) -> str:
    """Get board name, MicroPython version, and memory info.

    Args:
        keep_open: If true, keep the serial port open between calls.
    """
    code = """\
import sys, os, gc
gc.collect()
print("platform:", sys.platform)
print("version:", sys.version)
print("implementation:", sys.implementation)
uname = os.uname()
print("machine:", uname.machine)
print("sysname:", uname.sysname)
print("release:", uname.release)
gc.collect()
print("mem_free:", gc.mem_free())
print("mem_alloc:", gc.mem_alloc())
"""
    with connect(keep_open=keep_open) as t:
        result = t.exec(code)
        return result.decode(errors="replace")


if __name__ == "__main__":
    mcp.run(transport="stdio")
