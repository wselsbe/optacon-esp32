import asyncio
import json

import ota
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
        hz = data.get("hz", 250)
        amp = data.get("amplitude", 100)
        wf = data.get("waveform", "sine")
        fw = data.get("fullwave", False)
        dt = data.get("dead_time", 0)
        adv = data.get("phase_advance", 0)
        # Fast path: live update if only hz/amplitude/waveform changed
        if (was_running and pa._mode == "analog"
                and fw == pa._fullwave and dt == pa._dead_time
                and adv == pa._phase_advance):
            pa.set_frequency_live(hz, amplitude=amp, waveform=wf)
        else:
            if was_running:
                pa.stop()
            pa.set_frequency_analog(
                hz, amplitude=amp, fullwave=fw,
                dead_time=dt, phase_advance=adv, waveform=wf,
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
    elif cmd == "exec":
        code = data.get("code", "")
        output = []

        def _print(*args, **kw):
            sep = kw.get("sep", " ")
            end = kw.get("end", "\n")
            output.append(sep.join(str(a) for a in args) + end)

        g = {"print": _print}
        try:
            try:
                result = eval(code, g)
                if result is not None:
                    output.append(repr(result) + "\n")
            except SyntaxError:
                exec(code, g)
        except Exception as e:
            import io
            import sys

            buf = io.StringIO()
            sys.print_exception(e, buf)
            output.append(buf.getvalue())
        return {"output": "".join(output)}
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


# --- OTA API ---


@app.route("/api/ota/status")
async def ota_status(request):
    return json.dumps(ota.get_status()), 200, {"Content-Type": "application/json"}


@app.route("/api/ota/config", methods=["GET", "PUT"])
async def ota_config(request):
    if request.method == "GET":
        return json.dumps(ota.load_config()), 200, {"Content-Type": "application/json"}
    # PUT: update config
    data = json.loads(request.body.decode())
    cfg = ota.load_config()
    for key in ("update_url", "diagnostics_url", "auto_check"):
        if key in data:
            cfg[key] = data[key]
    ota.save_config(cfg)
    return json.dumps(cfg), 200, {"Content-Type": "application/json"}


@app.route("/api/ota/check", methods=["POST"])
async def ota_check(request):
    result = ota.check_for_updates()
    if result is None:
        return json.dumps({"error": "Check failed"}), 500, {"Content-Type": "application/json"}
    return json.dumps(result), 200, {"Content-Type": "application/json"}


@app.route("/api/ota/update/firmware", methods=["POST"])
async def ota_update_firmware(request):
    data = json.loads(request.body.decode())
    version = data.get("version")
    manifest = data.get("manifest")
    if not version or not manifest:
        return json.dumps({"error": "version and manifest required"}), 400, {
            "Content-Type": "application/json"
        }
    ok = ota.update_firmware(manifest, version)
    if ok:
        return (
            json.dumps({"status": "ok", "message": "Firmware updated. Rebooting..."}),
            200,
            {"Content-Type": "application/json"},
        )
    return json.dumps({"error": "Firmware update failed"}), 500, {"Content-Type": "application/json"}


@app.route("/api/ota/update/files", methods=["POST"])
async def ota_update_files(request):
    data = json.loads(request.body.decode())
    version = data.get("version")
    manifest = data.get("manifest")
    if not version or not manifest:
        return json.dumps({"error": "version and manifest required"}), 400, {
            "Content-Type": "application/json"
        }
    ok = ota.update_files(manifest, version)
    if ok:
        return (
            json.dumps({"status": "ok", "message": "Files updated. Rebooting..."}),
            200,
            {"Content-Type": "application/json"},
        )
    return json.dumps({"error": "File update failed"}), 500, {"Content-Type": "application/json"}


@app.route("/api/ota/upload", methods=["POST"])
async def ota_upload(request):
    filename = request.args.get("filename", "")
    if filename:
        path = "/" + filename.lstrip("/")
        ota.upload_file(path, request.body)
        return json.dumps({"status": "ok", "path": path}), 200, {"Content-Type": "application/json"}
    else:
        ok = ota.upload_firmware(request.body, len(request.body))
        if ok:
            return (
                json.dumps({"status": "ok", "message": "Firmware uploaded. Rebooting..."}),
                200,
                {"Content-Type": "application/json"},
            )
        return json.dumps({"error": "Upload failed"}), 500, {"Content-Type": "application/json"}


@app.route("/api/ota/log")
async def ota_log(request):
    name = request.args.get("file", "boot")
    content = ota.get_log(name)
    return json.dumps({"log": content}), 200, {"Content-Type": "application/json"}


@app.route("/api/ota/diagnostics", methods=["POST"])
async def ota_diagnostics(request):
    ok = ota.send_diagnostics()
    if ok:
        return json.dumps({"status": "ok"}), 200, {"Content-Type": "application/json"}
    return json.dumps({"error": "Failed to send diagnostics"}), 500, {
        "Content-Type": "application/json"
    }


@app.route("/update")
async def update_page(request):
    return send_file("/web/update.html")


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
