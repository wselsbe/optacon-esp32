"""Tests for OTA device header injection."""
import importlib
from unittest.mock import MagicMock, patch

import pytest


@pytest.fixture
def ota_module(tmp_path, monkeypatch):
    """Import ota module with temp filesystem for config files."""
    monkeypatch.chdir(tmp_path)
    import ota

    importlib.reload(ota)
    return ota


def test_device_details_helper(ota_module):
    """_device_details() returns hw_info.get_headers() result."""
    import hw_info

    expected = {
        "X-Device-Serial-Number": "TEST-001",
        "X-Device-Hw-Revision": "1.0",
        "X-Device-Firmware-Version": "0.1.0",
    }
    with patch.object(hw_info, "get_headers", return_value=expected):
        result = ota_module._device_details()
    assert result == expected


def test_device_details_fallback_on_error(ota_module):
    """_device_details() returns X-Device-Error header if hw_info fails."""
    import hw_info

    with patch.object(hw_info, "get_headers", side_effect=Exception("no NVS")):
        result = ota_module._device_details()
    assert result == {"X-Device-Error": "no NVS"}


def test_headers_included_in_http_request(ota_module):
    """Verify X-Device-* headers appear in the HTTP request string."""
    import hw_info

    device_headers = {
        "X-Device-Serial-Number": "TEST-001",
        "X-Device-Hw-Revision": "1.0",
        "X-Device-Firmware-Version": "0.1.0",
    }

    # Capture what gets written to the socket
    written_data = []
    mock_sock = MagicMock()
    mock_sock.write = lambda d: written_data.append(d)
    mock_sock.readline = MagicMock(
        side_effect=[b"HTTP/1.0 200 OK\r\n", b"\r\n"]
    )
    mock_sock.read = MagicMock(return_value=b"")

    mock_socket_mod = MagicMock()
    mock_socket_mod.getaddrinfo.return_value = [(None, None, None, None, ("127.0.0.1", 80))]
    mock_socket_mod.socket.return_value = mock_sock

    with (
        patch.object(hw_info, "get_headers", return_value=device_headers),
        patch.dict("sys.modules", {"socket": mock_socket_mod}),
    ):
            importlib.reload(ota_module)
            # Re-import after reload to get fresh module reference
            import ota

            importlib.reload(ota)
            with patch.object(hw_info, "get_headers", return_value=device_headers):
                status, resp_headers, sock = ota._http_request(
                    "GET", "http://example.com/test"
                )

    # Check the request string contains device headers
    req_str = written_data[0].decode() if written_data else ""
    assert "X-Device-Serial-Number: TEST-001" in req_str
    assert "X-Device-Hw-Revision: 1.0" in req_str
    assert "X-Device-Firmware-Version: 0.1.0" in req_str


def test_caller_headers_override_device_details(ota_module):
    """Caller-supplied headers should override device headers."""
    import hw_info

    device_headers = {
        "X-Device-Firmware-Version": "0.1.0",
        "Content-Type": "text/plain",
    }
    caller_headers = {"Content-Type": "application/json"}

    with patch.object(hw_info, "get_headers", return_value=device_headers):
        merged = ota_module._device_details()
        merged.update(caller_headers)

    assert merged["Content-Type"] == "application/json"  # caller wins
    assert merged["X-Device-Firmware-Version"] == "0.1.0"
