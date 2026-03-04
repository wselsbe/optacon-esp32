import asyncio
import json

import wifi
from microdot import Microdot, send_file
from microdot.websocket import with_websocket
from pz_drive_py import PzActuator

app = Microdot()
pa = PzActuator()


def _get_status():
    """Build full status dict."""
    status = pa.get_status()
    status.update(wifi.get_status())
    return status


def _handle_command(msg):
    """Dispatch a JSON command to PzActuator. Returns status dict."""
    data = json.loads(msg)
    cmd = data.get("cmd")

    was_running = pa.is_running()

    if cmd == "set_frequency_analog":
        if was_running:
            pa.stop()
        pa.set_frequency_analog(
            data.get("hz", 250),
            amplitude=data.get("amplitude", 100),
            fullwave=data.get("fullwave", False),
            dead_time=data.get("dead_time", 0),
            phase_advance=data.get("phase_advance", 0),
            waveform=data.get("waveform", "sine"),
        )
        if was_running:
            pa.start(gain=pa._gain)
    elif cmd == "set_frequency_digital":
        if was_running:
            pa.stop()
        pa.set_frequency_digital(
            data.get("hz", 100),
            fullwave=data.get("fullwave", False),
            waveform=data.get("waveform", "sine"),
        )
        if was_running:
            pa.start(gain=pa._gain)
    elif cmd == "start":
        if was_running:
            pa.stop()
        pa.start(gain=data.get("gain", 100))
    elif cmd == "stop":
        pa.stop()
    elif cmd == "set_pin":
        pa.shift_register.set_pin(data["pin"], data["value"])
    elif cmd == "set_all":
        pa.shift_register.set_all(data["value"])
    elif cmd == "set_polarity":
        import pz_drive

        pz_drive.pol_set(bool(data.get("value", False)))
    elif cmd == "get_status":
        pass  # status is always returned
    elif cmd == "wifi_config":
        wifi.save_config(data["ssid"], data.get("password", ""))
        wifi.reconnect()
        return {"msg": "WiFi reconnected", "ip": wifi.ip}
    else:
        return {"error": f"unknown command: {cmd}"}

    return None


@app.route("/")
async def index(request):
    return send_file("/web/index.html")


@app.route("/web/<path:path>")
async def static(request, path):
    if ".." in path:
        return "Not found", 404
    return send_file("/web/" + path)


@app.route("/wifi")
async def wifi_page(request):
    return send_file("/web/wifi.html")


@app.route("/wifi/status")
async def wifi_status(request):
    return json.dumps(wifi.get_status()), 200, {"Content-Type": "application/json"}


@app.route("/ws")
@with_websocket
async def websocket(request, ws):
    try:
        # Send initial status
        await ws.send(json.dumps(_get_status()))
        while True:
            msg = await ws.receive()
            try:
                err = _handle_command(msg)
                if err:
                    await ws.send(json.dumps(err))
                else:
                    await ws.send(json.dumps(_get_status()))
            except Exception as e:
                await ws.send(json.dumps({"error": str(e)}))
    except Exception as e:
        print("WebSocket error:", type(e).__name__, e)


def start():
    """Connect WiFi and start the web server (runs forever)."""
    wifi.connect()
    print("Starting web server on http://" + wifi.ip + ":80")
    asyncio.run(app.start_server(host="0.0.0.0", port=80))
