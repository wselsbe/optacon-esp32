"""E2E API and WebSocket tests for web_server.py."""

import json
from unittest.mock import MagicMock

import aiohttp
import pytest

# ---------------------------------------------------------------------------
# Smoke test (sync, no server needed)
# ---------------------------------------------------------------------------


def test_create_app_with_mock_deps():
    """web_server.create_app() returns a Microdot app with mock deps."""
    import web_server

    deps = MagicMock()
    deps.pa.get_status.return_value = {
        "running": False,
        "mode": None,
        "frequency": 0,
        "gain": 100,
        "fullwave": False,
        "waveform": "sine",
        "polarity": False,
        "pins": [0] * 20,
    }
    deps.wifi.get_status.return_value = {
        "mode": "ap",
        "ssid": "test",
        "ip": "192.168.4.1",
        "hostname": "test.local",
    }

    app = web_server.create_app(deps)
    assert app is not None
    assert hasattr(app, "start_server")


# ---------------------------------------------------------------------------
# HTTP API tests
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_wifi_status(test_app):
    """GET /wifi/status returns WiFi status JSON."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.get(f"{url}/wifi/status") as r:
        assert r.status == 200
        data = await r.json()
        assert data["mode"] == "sta"
        assert data["ip"] == "127.0.0.1"


@pytest.mark.asyncio
async def test_ota_status(test_app):
    """GET /api/ota/status returns OTA status JSON."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.get(f"{url}/api/ota/status") as r:
        assert r.status == 200
        data = await r.json()
        assert data["firmware_version"] == "0.1.0"


@pytest.mark.asyncio
async def test_ota_config_get(test_app):
    """GET /api/ota/config returns OTA config."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.get(f"{url}/api/ota/config") as r:
        assert r.status == 200
        data = await r.json()
        assert "update_url" in data
        assert "auto_check" in data


@pytest.mark.asyncio
async def test_ota_config_put(test_app):
    """PUT /api/ota/config updates config and calls save_config."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s:
        payload = {"update_url": "http://example.com/ota"}
        async with s.put(f"{url}/api/ota/config", data=json.dumps(payload)) as r:
            assert r.status == 200
            data = await r.json()
            assert data["update_url"] == "http://example.com/ota"
    deps.ota.save_config.assert_called_once()


@pytest.mark.asyncio
async def test_ota_check(test_app):
    """POST /api/ota/check returns update check result."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.post(f"{url}/api/ota/check") as r:
        assert r.status == 200
        data = await r.json()
        assert data["firmware_available"] is False


@pytest.mark.asyncio
async def test_ota_check_failure(test_app):
    """POST /api/ota/check returns 500 when check fails."""
    url, deps = test_app
    deps.ota.check_for_updates.return_value = None
    async with aiohttp.ClientSession() as s, s.post(f"{url}/api/ota/check") as r:
        assert r.status == 500
        data = await r.json()
        assert "error" in data


@pytest.mark.asyncio
async def test_ota_update_firmware(test_app):
    """POST /api/ota/update/firmware triggers firmware update."""
    url, deps = test_app
    payload = {"version": "0.2.0", "manifest": {"url": "http://example.com/fw.bin"}}
    async with aiohttp.ClientSession() as s, s.post(
        f"{url}/api/ota/update/firmware", data=json.dumps(payload)
    ) as r:
        assert r.status == 200
        data = await r.json()
        assert data["status"] == "ok"
    deps.ota.update_firmware.assert_called_once()


@pytest.mark.asyncio
async def test_ota_update_firmware_missing_params(test_app):
    """POST /api/ota/update/firmware without version returns 400."""
    url, deps = test_app
    payload = {"manifest": {"url": "http://example.com/fw.bin"}}
    async with aiohttp.ClientSession() as s, s.post(
        f"{url}/api/ota/update/firmware", data=json.dumps(payload)
    ) as r:
        assert r.status == 400
        data = await r.json()
        assert "error" in data


@pytest.mark.asyncio
async def test_ota_update_files(test_app):
    """POST /api/ota/update/files triggers file update."""
    url, deps = test_app
    payload = {"version": "0.2.0", "manifest": {"changes": []}}
    async with aiohttp.ClientSession() as s, s.post(
        f"{url}/api/ota/update/files", data=json.dumps(payload)
    ) as r:
        assert r.status == 200
        data = await r.json()
        assert data["status"] == "ok"
    deps.ota.update_files.assert_called_once()


@pytest.mark.asyncio
async def test_ota_upload_file(test_app):
    """POST /api/ota/upload?filename=test.py uploads a file."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.post(
        f"{url}/api/ota/upload?filename=test.py", data=b"print('hello')"
    ) as r:
        assert r.status == 200
        data = await r.json()
        assert data["status"] == "ok"
        assert data["path"] == "/test.py"
    deps.ota.upload_file.assert_called_once()


@pytest.mark.asyncio
async def test_ota_upload_firmware(test_app):
    """POST /api/ota/upload (no filename) uploads firmware."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.post(f"{url}/api/ota/upload", data=b"\x00" * 100) as r:
        assert r.status == 200
        data = await r.json()
        assert data["status"] == "ok"
    deps.ota.upload_firmware.assert_called_once()


@pytest.mark.asyncio
async def test_ota_log(test_app):
    """GET /api/ota/log returns log content."""
    url, deps = test_app
    deps.ota.get_log.return_value = "boot ok"
    async with aiohttp.ClientSession() as s, s.get(f"{url}/api/ota/log") as r:
        assert r.status == 200
        data = await r.json()
        assert data["log"] == "boot ok"


@pytest.mark.asyncio
async def test_ota_diagnostics(test_app):
    """POST /api/ota/diagnostics sends diagnostics successfully."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.post(f"{url}/api/ota/diagnostics") as r:
        assert r.status == 200
        data = await r.json()
        assert data["status"] == "ok"


@pytest.mark.asyncio
async def test_ota_diagnostics_failure(test_app):
    """POST /api/ota/diagnostics returns 500 on failure."""
    url, deps = test_app
    deps.ota.send_diagnostics.return_value = False
    async with aiohttp.ClientSession() as s, s.post(f"{url}/api/ota/diagnostics") as r:
        assert r.status == 500
        data = await r.json()
        assert "error" in data


@pytest.mark.asyncio
async def test_music_songs(test_app):
    """GET /api/music/songs returns song list."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.get(f"{url}/api/music/songs") as r:
        assert r.status == 200
        data = await r.json()
        assert "songs" in data
        assert isinstance(data["songs"], list)
        assert len(data["songs"]) > 0
        # Each song has expected fields
        song = data["songs"][0]
        assert "name" in song
        assert "notes" in song
        assert "bpm" in song


# ---------------------------------------------------------------------------
# WebSocket tests
# ---------------------------------------------------------------------------


def _ws_url(http_url):
    """Convert http://host:port to ws://host:port/ws."""
    return http_url.replace("http://", "ws://") + "/ws"


@pytest.mark.asyncio
async def test_ws_initial_status(test_app):
    """WebSocket connection receives initial status message."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        msg = await ws.receive_json()
        assert "running" in msg
        assert "pins" in msg
        assert "frequency" in msg


@pytest.mark.asyncio
async def test_ws_get_status(test_app):
    """WebSocket get_status command returns status dict."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        await ws.receive_json()  # initial status
        await ws.send_json({"cmd": "get_status"})
        msg = await ws.receive_json()
        assert "running" in msg
        assert "frequency" in msg


@pytest.mark.asyncio
async def test_ws_start(test_app):
    """WebSocket start command calls deps.pa.start."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        await ws.receive_json()  # initial status
        await ws.send_json({"cmd": "start", "gain": 75})
        await ws.receive_json()  # status response
    deps.pa.start.assert_called_once_with(gain=75)


@pytest.mark.asyncio
async def test_ws_stop(test_app):
    """WebSocket stop command calls deps.pa.stop."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        await ws.receive_json()  # initial status
        await ws.send_json({"cmd": "stop"})
        await ws.receive_json()  # status response
    deps.pa.stop.assert_called()


@pytest.mark.asyncio
async def test_ws_set_frequency_analog(test_app):
    """WebSocket set_frequency_analog calls deps.pa.set_frequency_analog."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        await ws.receive_json()  # initial status
        await ws.send_json({
            "cmd": "set_frequency_analog",
            "hz": 440,
            "amplitude": 80,
            "waveform": "triangle",
            "fullwave": True,
            "dead_time": 5,
            "phase_advance": 10,
        })
        await ws.receive_json()  # status response
    # Not running, so it takes the non-live path
    deps.pa.set_frequency_analog.assert_called_once_with(
        440, amplitude=80, fullwave=True, dead_time=5, phase_advance=10, waveform="triangle"
    )


@pytest.mark.asyncio
async def test_ws_set_frequency_digital(test_app):
    """WebSocket set_frequency_digital calls deps.pa.set_frequency_digital."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        await ws.receive_json()  # initial status
        await ws.send_json({
            "cmd": "set_frequency_digital",
            "hz": 200,
            "fullwave": True,
            "waveform": "square",
        })
        await ws.receive_json()  # status response
    deps.pa.set_frequency_digital.assert_called_once_with(
        200, fullwave=True, waveform="square"
    )


@pytest.mark.asyncio
async def test_ws_set_pin(test_app):
    """WebSocket set_pin calls shift_register.set_pin."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        await ws.receive_json()  # initial status
        await ws.send_json({"cmd": "set_pin", "pin": 5, "value": 1})
        await ws.receive_json()  # status response
    deps.pa.shift_register.set_pin.assert_called_once_with(5, 1)


@pytest.mark.asyncio
async def test_ws_set_all(test_app):
    """WebSocket set_all calls shift_register.set_all."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        await ws.receive_json()  # initial status
        await ws.send_json({"cmd": "set_all", "value": 1})
        await ws.receive_json()  # status response
    deps.pa.shift_register.set_all.assert_called_once_with(1)


@pytest.mark.asyncio
async def test_ws_set_polarity(test_app):
    """WebSocket set_polarity calls pz_drive.pol_set."""
    import pz_drive

    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        await ws.receive_json()  # initial status
        await ws.send_json({"cmd": "set_polarity", "value": True})
        await ws.receive_json()  # status response
    pz_drive.pol_set.assert_called_once_with(True)


@pytest.mark.asyncio
async def test_ws_say(test_app):
    """WebSocket say command returns speech complete."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        await ws.receive_json()  # initial status
        await ws.send_json({"cmd": "say", "text": "hello"})
        msg = await ws.receive_json()
        assert msg["msg"] == "speech complete"


@pytest.mark.asyncio
async def test_ws_play_music(test_app):
    """WebSocket play_music command returns music complete."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        await ws.receive_json()  # initial status
        await ws.send_json({
            "cmd": "play_music",
            "notes": "C4:1 E4:1 G4:2",
            "bpm": 120,
        })
        msg = await ws.receive_json()
        assert msg["msg"] == "music complete"


@pytest.mark.asyncio
async def test_ws_play_song(test_app):
    """WebSocket play_song command plays a known song."""
    import music

    song_name = next(iter(music.SONGS.keys()))
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        await ws.receive_json()  # initial status
        await ws.send_json({"cmd": "play_song", "name": song_name})
        msg = await ws.receive_json()
        assert msg["msg"] == "music complete"


@pytest.mark.asyncio
async def test_ws_exec(test_app):
    """WebSocket exec command evaluates code and returns output."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        await ws.receive_json()  # initial status
        await ws.send_json({"cmd": "exec", "code": "2+2"})
        msg = await ws.receive_json()
        assert "4" in msg["output"]


@pytest.mark.asyncio
async def test_ws_unknown_command(test_app):
    """WebSocket unknown command returns error."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        await ws.receive_json()  # initial status
        await ws.send_json({"cmd": "nonexistent_command"})
        msg = await ws.receive_json()
        assert "error" in msg


@pytest.mark.asyncio
async def test_ws_wifi_config(test_app):
    """WebSocket wifi_config command calls deps.wifi.save_config."""
    url, deps = test_app
    async with aiohttp.ClientSession() as s, s.ws_connect(_ws_url(url)) as ws:
        await ws.receive_json()  # initial status
        await ws.send_json({
            "cmd": "wifi_config",
            "ssid": "MyNetwork",
            "password": "secret123",
        })
        msg = await ws.receive_json()
        assert "msg" in msg
    deps.wifi.save_config.assert_called_once_with("MyNetwork", "secret123")
    deps.wifi.reconnect.assert_called_once()
