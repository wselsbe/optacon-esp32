import asyncio
import json


def _schedule_reset():
    """Schedule reboot via timer so HTTP response can flush first."""
    from machine import Timer, reset

    Timer(0).init(period=1000, mode=Timer.ONE_SHOT, callback=lambda t: reset())


class _Deps:
    """Default dependency container with real hardware."""

    def __init__(self):
        import ota as _ota
        import wifi as _wifi
        from pz_drive_py import PzDrive

        self.pa = PzDrive()
        self.ota = _ota
        self.wifi = _wifi


def create_app(deps=None):
    """Create and return a Microdot app with all routes registered.

    Args:
        deps: Optional dependency container. If None, creates _Deps()
              with real hardware. For testing, pass a mock object with
              .pa, .ota, and .wifi attributes.
    """
    if deps is None:
        deps = _Deps()

    from microdot import Microdot, send_file
    from microdot.websocket import with_websocket

    app = Microdot()

    def _parse_json(request):
        try:
            return json.loads(request.body.decode()), None
        except (ValueError, UnicodeDecodeError):
            return None, (json.dumps({"error": "invalid JSON"}), 400, {"Content-Type": "application/json"})

    def _get_status():
        """Build full status dict."""
        status = deps.pa.get_status()
        status.update(deps.wifi.get_status())
        return status

    def _handle_command(msg):
        """Dispatch a JSON command to PzDrive. Returns status dict."""
        data = json.loads(msg)
        cmd = data.get("cmd")

        was_running = deps.pa.is_running()

        if cmd == "set_frequency_analog":
            hz = data.get("hz", 250)
            amp = data.get("amplitude", 100)
            wf = data.get("waveform", "sine")
            fw = data.get("fullwave", False)
            dt = data.get("dead_time", 0)
            adv = data.get("phase_advance", 0)
            # Fast path: live update if only hz/amplitude/waveform changed
            if (was_running and deps.pa._mode == "analog"
                    and fw == deps.pa._fullwave and dt == deps.pa._dead_time
                    and adv == deps.pa._phase_advance):
                deps.pa.set_frequency_live(hz, amplitude=amp, waveform=wf)
            else:
                if was_running:
                    deps.pa.stop()
                deps.pa.set_frequency_analog(
                    hz, amplitude=amp, fullwave=fw,
                    dead_time=dt, phase_advance=adv, waveform=wf,
                )
                if was_running:
                    deps.pa.start(gain=deps.pa._gain)
        elif cmd == "set_frequency_digital":
            if was_running:
                deps.pa.stop()
            deps.pa.set_frequency_digital(
                data.get("hz", 100),
                fullwave=data.get("fullwave", False),
                waveform=data.get("waveform", "sine"),
            )
            if was_running:
                deps.pa.start(gain=deps.pa._gain)
        elif cmd == "start":
            if was_running:
                deps.pa.stop()
            deps.pa.start(gain=data.get("gain", 100))
        elif cmd == "stop":
            deps.pa.stop()
        elif cmd == "set_pin":
            pin = data.get("pin")
            value = data.get("value")
            if pin is None or value is None:
                return {"error": "pin and value required"}
            deps.pa.set_pin(pin, value)
        elif cmd == "set_all":
            value = data.get("value")
            if value is None:
                return {"error": "value required"}
            deps.pa.set_all(value)
        elif cmd == "set_polarity":
            import pz_drive

            pz_drive.pol_set(bool(data.get("value", False)))
        elif cmd == "get_status":
            pass  # status is always returned
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

    # --- OTA API ---

    @app.route("/api/ota/config", methods=["GET", "PUT"])
    async def ota_config(request):
        if request.method == "GET":
            return json.dumps(deps.ota.load_config()), 200, {"Content-Type": "application/json"}
        # PUT: update config
        data, err = _parse_json(request)
        if err:
            return err
        cfg = deps.ota.load_config()
        for key in ("update_url", "auto_check"):
            if key in data:
                cfg[key] = data[key]
        deps.ota.save_config(cfg)
        return json.dumps(cfg), 200, {"Content-Type": "application/json"}

    @app.route("/api/ota/check", methods=["POST"])
    async def ota_check(request):
        result = deps.ota.check_for_updates()
        if result is None:
            return json.dumps({"error": "Check failed"}), 500, {"Content-Type": "application/json"}
        return json.dumps(result), 200, {"Content-Type": "application/json"}

    @app.route("/api/ota/update/firmware", methods=["POST"])
    async def ota_update_firmware(request):
        data, err = _parse_json(request)
        if err:
            return err
        version = data.get("version")
        manifest = data.get("manifest")
        if not version or not manifest:
            return json.dumps({"error": "version and manifest required"}), 400, {
                "Content-Type": "application/json"
            }
        ok = deps.ota.update_firmware(manifest, version)
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
        data, err = _parse_json(request)
        if err:
            return err
        version = data.get("version")
        manifest = data.get("manifest")
        if not version or not manifest:
            return json.dumps({"error": "version and manifest required"}), 400, {
                "Content-Type": "application/json"
            }
        ok = deps.ota.update_files(manifest, version)
        if ok:
            _schedule_reset()
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
            deps.ota.upload_file(path, request.body)
            return json.dumps({"status": "ok", "path": path}), 200, {"Content-Type": "application/json"}
        else:
            ok = deps.ota.upload_firmware(request.body, len(request.body))
            if ok:
                _schedule_reset()
                return (
                    json.dumps({"status": "ok", "message": "Firmware uploaded. Rebooting..."}),
                    200,
                    {"Content-Type": "application/json"},
                )
            return json.dumps({"error": "Upload failed"}), 500, {"Content-Type": "application/json"}

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

    # --- Device API ---

    @app.route("/api/device/status")
    async def device_status(request):
        return json.dumps(deps.ota.get_status()), 200, {"Content-Type": "application/json"}

    @app.route("/api/device/log")
    async def device_log(request):
        name = request.args.get("file", "boot")
        content = deps.ota.get_log(name)
        return json.dumps({"log": content}), 200, {"Content-Type": "application/json"}

    @app.route("/api/device/diagnostics", methods=["POST"])
    async def device_diagnostics(request):
        ok = deps.ota.send_diagnostics()
        if ok:
            return json.dumps({"status": "ok"}), 200, {"Content-Type": "application/json"}
        return json.dumps({"error": "Failed to send diagnostics"}), 500, {
            "Content-Type": "application/json"
        }

    # --- WiFi API ---

    @app.route("/api/wifi/status")
    async def wifi_status(request):
        return json.dumps(deps.wifi.get_status()), 200, {"Content-Type": "application/json"}

    @app.route("/api/wifi/config", methods=["PUT"])
    async def wifi_config(request):
        data, err = _parse_json(request)
        if err:
            return err
        ssid = data.get("ssid", "")
        if not ssid:
            return json.dumps({"error": "ssid required"}), 400, {"Content-Type": "application/json"}
        deps.wifi.save_config(ssid, data.get("password", ""))
        deps.wifi.reconnect()
        return json.dumps({"status": "ok", "ip": deps.wifi.ip}), 200, {
            "Content-Type": "application/json"
        }

    # --- TTS API ---

    @app.route("/api/tts/say", methods=["POST"])
    async def tts_say(request):
        import sam

        data, err = _parse_json(request)
        if err:
            return err
        text = data.get("text", "")
        if not text:
            return json.dumps({"error": "no text provided"}), 400, {
                "Content-Type": "application/json"
            }
        # NOTE: sam.say() is a blocking C module call that will block the async
        # event loop for the duration of speech synthesis + playback. This is an
        # inherent limitation — SAM renders and plays audio synchronously via the
        # pz_drive ISR. No async workaround is possible without C-level changes.
        sam.say(
            text,
            speed=data.get("speed", 72),
            pitch=data.get("pitch", 64),
            mouth=data.get("mouth", 128),
            throat=data.get("throat", 128),
        )
        return json.dumps({"status": "ok"}), 200, {"Content-Type": "application/json"}

    # --- Music API (play/play_song) ---

    @app.route("/api/music/play", methods=["POST"])
    async def music_play(request):
        import music

        data, err = _parse_json(request)
        if err:
            return err
        notes_str = data.get("notes", "")
        if not notes_str:
            return json.dumps({"error": "no notes provided"}), 400, {
                "Content-Type": "application/json"
            }
        song = []
        for token in notes_str.split():
            parts = token.split(":")
            note = parts[0]
            try:
                beats = float(parts[1]) if len(parts) > 1 else 1
            except ValueError:
                return json.dumps({"error": "invalid beat value: " + parts[1]}), 400, {
                    "Content-Type": "application/json"
                }
            song.append((note if note != "R" else "R", beats))
        # NOTE: music.play() blocks the async event loop for the entire song
        # duration. It uses time.sleep_ms() internally for note timing and drives
        # the piezo via pz_drive C calls. No async workaround is feasible without
        # rewriting the player to yield between notes.
        music.play(
            song,
            bpm=data.get("bpm", 120),
            waveform=data.get("waveform", "sine"),
            gain=data.get("gain", 75),
        )
        return json.dumps({"status": "ok"}), 200, {"Content-Type": "application/json"}

    @app.route("/api/music/play_song", methods=["POST"])
    async def music_play_song(request):
        import music

        data, err = _parse_json(request)
        if err:
            return err
        name = data.get("name", "")
        if name not in music.SONGS:
            return json.dumps({"error": "unknown song: " + name}), 404, {
                "Content-Type": "application/json"
            }
        # NOTE: music.play_song() blocks the event loop (see music.play note above).
        music.play_song(
            name,
            waveform=data.get("waveform", "sine"),
        )
        return json.dumps({"status": "ok"}), 200, {"Content-Type": "application/json"}

    # --- Exec API ---

    @app.route("/api/exec", methods=["POST"])
    async def exec_code(request):
        data, err = _parse_json(request)
        if err:
            return err
        code = data.get("code", "")
        output = []

        def _print(*args, **kw):
            sep = kw.get("sep", " ")
            end = kw.get("end", "\n")
            output.append(sep.join(str(a) for a in args) + end)

        g = {"print": _print, "pa": deps.pa}
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
            try:
                sys.print_exception(e, buf)
            except AttributeError:
                import traceback

                traceback.print_exception(type(e), e, e.__traceback__, file=buf)
            output.append(buf.getvalue())
        return json.dumps({"output": "".join(output)}), 200, {"Content-Type": "application/json"}

    @app.route("/api")
    async def api_page(request):
        return send_file("/web/api.html")

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

    return app


async def _ota_auto_check():
    """Background OTA version check — runs after server is already listening."""
    import ota

    await asyncio.sleep(2)  # let server settle before HTTP call
    try:
        cfg = ota.load_config()
        if not cfg.get("auto_check", True):
            return
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


async def _start_async():
    """Start web server and schedule background OTA check."""
    import wifi

    app = create_app()
    server = asyncio.create_task(app.start_server(host="0.0.0.0", port=80))
    if wifi.mode == "sta":
        asyncio.create_task(_ota_auto_check())
    await server


def start():
    """Connect WiFi and start the web server (runs forever)."""
    import wifi

    wifi.connect()
    print("Starting web server on http://" + wifi.ip + ":80")
    asyncio.run(_start_async())
