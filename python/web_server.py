import asyncio
import json
import time

import machine
import network
from microdot import Microdot, send_file
from microdot.websocket import with_websocket
from pz_actuator_py import PzActuator

app = Microdot()
pa = PzActuator()

HOSTNAME = "esp-optacon"
STA_TIMEOUT = 10  # seconds to wait for station connection
AP_SSID = "esp-optacon"
AP_PASSWORD = "123456"


def _connect_wifi():
    """Try STA mode from wifi_config.json, fall back to AP. Sets mDNS hostname."""
    network.hostname(HOSTNAME)

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
                print("Web UI: http://" + HOSTNAME + ".local")
                return ip, "sta", cfg["ssid"]
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
    print("Web UI: http://" + HOSTNAME + ".local")
    return ip, "ap", AP_SSID


def _get_status(ip=None):
    """Build full status dict."""
    status = pa.get_status()
    if ip:
        status["ip"] = ip
    status["hostname"] = HOSTNAME + ".local"
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
    elif cmd == "wifi_config":
        with open("wifi_config.json", "w") as f:
            json.dump({"ssid": data["ssid"], "password": data.get("password", "")}, f)
        return {"msg": "WiFi config saved. Rebooting...", "reboot": True}
    else:
        return {"error": f"unknown command: {cmd}"}

    return None


_server_ip = None
_wifi_mode = None
_wifi_ssid = None


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
    info = {
        "mode": _wifi_mode,
        "ssid": _wifi_ssid,
        "ip": _server_ip,
        "hostname": HOSTNAME + ".local",
    }
    return json.dumps(info), 200, {"Content-Type": "application/json"}


@app.route("/ws")
@with_websocket
async def websocket(request, ws):
    try:
        # Send initial status
        await ws.send(json.dumps(_get_status(_server_ip)))
        while True:
            msg = await ws.receive()
            try:
                err = _handle_command(msg)
                if err:
                    await ws.send(json.dumps(err))
                    if err.get("reboot"):
                        await asyncio.sleep(1)
                        machine.reset()
                else:
                    await ws.send(json.dumps(_get_status(_server_ip)))
            except Exception as e:
                await ws.send(json.dumps({"error": str(e)}))
    except Exception as e:
        print("WebSocket error:", type(e).__name__, e)


def start():
    """Connect WiFi and start the web server (runs forever)."""
    global _server_ip, _wifi_mode, _wifi_ssid
    _server_ip, _wifi_mode, _wifi_ssid = _connect_wifi()
    print("Starting web server on http://" + _server_ip + ":80")
    asyncio.run(app.start_server(host="0.0.0.0", port=80))
