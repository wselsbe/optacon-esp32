"""MCP server for MicroPython boards via mpremote serial transport."""

import logging
import os
import stat
import sys
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


@contextmanager
def connect(soft_reset=False):
    """Connect to device, yield transport, always close."""
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
def exec(code: str) -> str:
    """Execute MicroPython code on the board and return stdout.

    Args:
        code: Python code to execute on the device.
    """
    with connect() as t:
        result = t.exec(code)
        return result.decode(errors="replace")


@mcp.tool()
def soft_reset() -> str:
    """Soft-reset the MicroPython board (equivalent to Ctrl-D)."""
    with connect(soft_reset=True) as t:
        result = t.exec("print('reset ok')")
        return result.decode(errors="replace")


@mcp.tool()
def list_files(path: str = "/") -> str:
    """List files and directories on the device filesystem.

    Args:
        path: Directory path to list (default: root "/").
    """
    with connect() as t:
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
def read_file(path: str) -> str:
    """Read a file from the device filesystem.

    Args:
        path: File path on the device (e.g. "/main.py").
    """
    with connect() as t:
        data = t.fs_readfile(path)
        return data.decode(errors="replace")


@mcp.tool()
def write_file(path: str, content: str) -> str:
    """Write content to a file on the device filesystem.

    Args:
        path: Destination file path on the device (e.g. "/main.py").
        content: File content to write.
    """
    with connect() as t:
        t.fs_writefile(path, content.encode())
        return f"Wrote {len(content)} bytes to {path}"


@mcp.tool()
def device_info() -> str:
    """Get board name, MicroPython version, and memory info."""
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
    with connect() as t:
        result = t.exec(code)
        return result.decode(errors="replace")


if __name__ == "__main__":
    mcp.run(transport="stdio")
