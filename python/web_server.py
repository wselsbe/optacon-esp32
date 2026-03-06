import asyncio
import json

import ota
import wifi
from microdot import Microdot, send_file
from microdot.websocket import with_websocket
from pz_drive_py import PzActuator

app = Microdot()
pa = PzActuator()


def _schedule_reset():
    """Schedule reboot via timer so HTTP response can flush first."""
    from machine import Timer, reset

    Timer(0).init(period=1000, mode=Timer.ONE_SHOT, callback=lambda t: reset())


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
    elif cmd == "say":
        import sam

        text = data.get("text", "")
        if not text:
            return {"error": "no text provided"}
        sam.say(
            text,
            speed=data.get("speed", 72),
            pitch=data.get("pitch", 64),
            mouth=data.get("mouth", 128),
            throat=data.get("throat", 128),
        )
        return {"msg": "speech complete"}
    elif cmd == "play_music":
        import music

        notes_str = data.get("notes", "")
        if not notes_str:
            return {"error": "no notes provided"}
        song = []
        for token in notes_str.split():
            parts = token.split(":")
            note = parts[0]
            try:
                beats = float(parts[1]) if len(parts) > 1 else 1
            except ValueError:
                return {"error": "invalid beat value: " + parts[1]}
            song.append((note if note != "R" else "R", beats))
        music.play(
            song,
            bpm=data.get("bpm", 120),
            waveform=data.get("waveform", "sine"),
            gain=data.get("gain", 75),
        )
        return {"msg": "music complete"}
    elif cmd == "play_song":
        import music

        name = data.get("name", "")
        if name not in music.SONGS:
            return {"error": "unknown song: " + name}
        music.play_song(
            name,
            waveform=data.get("waveform", "sine"),
        )
        return {"msg": "music complete"}
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
        # Schedule reboot after response is sent
        _schedule_reset()
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
        if ".." in filename:
            return json.dumps({"error": "Invalid filename"}), 400, {"Content-Type": "application/json"}
        path = "/" + filename.lstrip("/")
        ota.upload_file(path, request.body)
        return json.dumps({"status": "ok", "path": path}), 200, {"Content-Type": "application/json"}
    else:
        ok = ota.upload_firmware(request.body, len(request.body))
        if ok:
            _schedule_reset()
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


@app.route("/api/music/songs")
async def music_songs(request):
    import music

    songs = []
    for name, (data, bpm, gain) in music.SONGS.items():
        notes = " ".join(
            f"{t[0]}:{t[1]}" for t in data
        )
        songs.append({"name": name, "notes": notes, "bpm": bpm, "gain": gain})
    return json.dumps({"songs": songs}), 200, {"Content-Type": "application/json"}


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

    # OTA auto-check after WiFi connects (station mode only)
    if wifi.mode == "sta":
        try:
            cfg = ota.load_config()
            if cfg.get("auto_check", True):
                result = ota.check_for_updates()
                if result and (result["firmware_available"] or result["files_available"]):
                    parts = []
                    if result["firmware_available"]:
                        parts.append("firmware " + result["latest_firmware"])
                    if result["files_available"]:
                        parts.append("files " + result["latest_files"])
                    msg = "OTA auto-check: update available (" + ", ".join(parts) + ")"
                    print("[BOOT]", msg)
                    try:
                        with open("/boot.log", "a") as f:
                            f.write(msg + "\n")
                    except Exception:
                        pass
        except Exception as e:
            print("[BOOT] OTA auto-check failed:", e)

    asyncio.run(app.start_server(host="0.0.0.0", port=80))
