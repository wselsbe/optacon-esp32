import json
import time
import asyncio
import network

from microdot import Microdot, send_file
from microdot.websocket import with_websocket
from pz_actuator_py import PzActuator

app = Microdot()
pa = PzActuator()

STA_TIMEOUT = 10  # seconds to wait for station connection
AP_SSID = "Optacon"
AP_PASSWORD = ""  # open network


def _connect_wifi():
    """Try STA mode from wifi_config.json, fall back to AP."""
    ip = None

    # Try station mode
    try:
        with open("wifi_config.json") as f:
            cfg = json.load(f)
        sta = network.WLAN(network.STA_IF)
        sta.active(True)
        sta.connect(cfg["ssid"], cfg.get("password", ""))
        for _ in range(STA_TIMEOUT * 10):
            if sta.isconnected():
                ip = sta.ifconfig()[0]
                print("WiFi STA connected:", ip)
                return ip
            time.sleep_ms(100)
        sta.active(False)
        print("WiFi STA failed, starting AP")
    except OSError:
        print("No wifi_config.json, starting AP")

    # Fallback: access point
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=AP_SSID, password=AP_PASSWORD)
    ip = ap.ifconfig()[0]
    print("WiFi AP started:", AP_SSID, "IP:", ip)
    return ip


def _get_status(ip=None):
    """Build full status dict."""
    status = pa.get_status()
    if ip:
        status["ip"] = ip
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
        pa.set_pin(data["pin"], data["value"])
    elif cmd == "set_all":
        pa.set_all(data["value"])
    elif cmd == "get_status":
        pass  # status is always returned
    else:
        return {"error": f"unknown command: {cmd}"}

    return None


_server_ip = None


@app.route("/")
async def index(request):
    return send_file("/web/index.html")


@app.route("/web/<path:path>")
async def static(request, path):
    if ".." in path:
        return "Not found", 404
    return send_file("/web/" + path)


@app.route("/ws")
@with_websocket
async def websocket(request, ws):
    # Send initial status
    await ws.send(json.dumps(_get_status(_server_ip)))
    try:
        while True:
            msg = await ws.receive()
            try:
                err = _handle_command(msg)
                if err:
                    await ws.send(json.dumps(err))
                else:
                    await ws.send(json.dumps(_get_status(_server_ip)))
            except Exception as e:
                await ws.send(json.dumps({"error": str(e)}))
    except asyncio.CancelledError:
        pass


def start():
    """Connect WiFi and start the web server (blocks)."""
    global _server_ip
    _server_ip = _connect_wifi()
    print("Starting web server on http://{}:80".format(_server_ip))
    app.run(host="0.0.0.0", port=80)
