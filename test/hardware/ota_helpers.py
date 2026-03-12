"""OTA hardware test helpers: board polling, IP discovery, log assertions."""

import logging
import socket
import time
import urllib.request

_log = logging.getLogger("ota.helpers")


def get_local_ip() -> str:
    """Get this machine's LAN IP via hostname resolution."""
    hostname = socket.gethostname()
    ip = socket.gethostbyname(hostname)
    if ip.startswith("127."):
        # Fallback: connect to a public DNS to discover LAN IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
        finally:
            s.close()
    _log.info("local IP: %s", ip)
    return ip


def wait_for_board(base_url: str, timeout: float = 60.0) -> bool:
    """Poll board's /api/device/status until responsive or timeout.

    Returns True when board responds with HTTP 200.
    Raises TimeoutError if timeout is exceeded.
    """
    url = base_url.rstrip("/") + "/api/device/status"
    deadline = time.monotonic() + timeout
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        try:
            req = urllib.request.urlopen(url, timeout=5)
            if req.status == 200:
                _log.info("board responsive after %d attempts", attempt)
                return True
        except Exception:
            pass
        time.sleep(1)
    raise TimeoutError(f"Board at {base_url} did not respond within {timeout}s")


def assert_log_sequence(log_content: str, patterns: list[str], log_name: str = "log"):
    """Assert that patterns appear in order within log content.

    Each pattern is a substring (not regex). Searches for each pattern
    starting from where the previous pattern was found.

    Raises AssertionError with details on failure.
    """
    pos = 0
    for i, pattern in enumerate(patterns):
        idx = log_content.find(pattern, pos)
        if idx == -1:
            context_start = max(0, pos - 100)
            context = log_content[context_start:pos + 200]
            raise AssertionError(
                f"Log sequence broken at step {i + 1}/{len(patterns)}: "
                f"pattern '{pattern}' not found in {log_name} after position {pos}.\n"
                f"Remaining log context: ...{context}..."
            )
        pos = idx + len(pattern)


def assert_log_absent(log_content: str, patterns: list[str], log_name: str = "log"):
    """Assert that none of the patterns appear in the log."""
    for pattern in patterns:
        if pattern in log_content:
            raise AssertionError(
                f"Unexpected pattern '{pattern}' found in {log_name}"
            )
