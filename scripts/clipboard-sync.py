#!/usr/bin/env python3
"""Bidirectional clipboard sync between Pi (Wayland) and Windows over SSH tunnels.

Two components run in parallel:

  Receiver (Windows→Pi): HTTP server on :8224 accepts POSTs from the Windows
  clipboard watcher and sets the Pi clipboard via wl-copy.

  Sender (Pi→Windows): Polls wl-paste every 500ms and POSTs changes to the
  Windows clipboard server on :8225.
"""

import hashlib
import subprocess
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer

RECV_PORT = 8224  # Windows→Pi (SSH -L on Windows side)
SEND_PORT = 8225  # Pi→Windows (SSH -R on Windows side)
SEND_URL = f"http://127.0.0.1:{SEND_PORT}/clipboard"
POLL_INTERVAL = 0.5

# Tracks the last hash we SET via the receiver, so the sender doesn't
# echo it back to Windows immediately.
_last_received_hash = None
_lock = threading.Lock()


# --- Receiver: Windows → Pi ------------------------------------------------


class ClipHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        global _last_received_hash
        content_type = self.headers.get("Content-Type", "text/plain")
        length = int(self.headers.get("Content-Length", 0))
        data = self.rfile.read(length)
        h = hashlib.sha256(data).hexdigest()
        with _lock:
            _last_received_hash = h
        try:
            # wl-copy forks into background to serve the selection, so use
            # --paste-once to make it exit after the first paste, preventing
            # it from blocking future wl-paste calls.
            proc = subprocess.Popen(
                ["wl-copy", "--type", content_type],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            proc.communicate(input=data, timeout=5)
            self.send_response(200 if proc.returncode == 0 else 500)
        except Exception as e:
            print(f"wl-copy error: {e}", file=sys.stderr)
            self.send_response(500)
        self.end_headers()

    def log_message(self, format, *args):
        pass


def run_receiver():
    server = HTTPServer(("127.0.0.1", RECV_PORT), ClipHandler)
    print(f"Receiver listening on 127.0.0.1:{RECV_PORT}")
    server.serve_forever()


# --- Sender: Pi → Windows --------------------------------------------------


def _run_timeout(cmd, timeout=2):
    """Run a command with hard timeout. Returns stdout bytes or None."""
    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        try:
            out, _ = proc.communicate(timeout=timeout)
            return out if proc.returncode == 0 else None
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
            return None
    except Exception:
        return None


def _get_wayland_clipboard():
    """Read Wayland clipboard. Returns (data, mime_type) or (None, None)."""
    types_raw = _run_timeout(["wl-paste", "--list-types"])
    if not types_raw:
        return None, None
    types = types_raw.decode().strip().splitlines()

    for t in types:
        if t.startswith("image/"):
            data = _run_timeout(["wl-paste", "--type", t, "--no-newline"])
            if data:
                return data, t
            break

    data = _run_timeout(["wl-paste", "--no-newline"])
    if data:
        return data, "text/plain;charset=utf-8"
    return None, None


def _get_x11_clipboard():
    """Read X11 clipboard (used by Claude Code /copy). Returns (data, mime) or (None, None)."""
    data = _run_timeout(["xclip", "-selection", "clipboard", "-o"], timeout=1)
    if data:
        return data, "text/plain;charset=utf-8"
    return None, None


def _send_to_windows(data, mime):
    """POST clipboard data to the Windows clipboard server."""
    req = urllib.request.Request(
        SEND_URL,
        data=data,
        headers={"Content-Type": mime},
        method="POST",
    )
    urllib.request.urlopen(req, timeout=3)


def run_sender():
    last_wl_hash = None
    last_x11_hash = None
    print(f"Sender polling clipboard → 127.0.0.1:{SEND_PORT}")
    while True:
        time.sleep(POLL_INTERVAL)
        try:
            # Check Wayland clipboard
            data, mime = _get_wayland_clipboard()
            if data:
                h = hashlib.sha256(data).hexdigest()
                with _lock:
                    is_echo = h == _last_received_hash
                if h != last_wl_hash and not is_echo:
                    last_wl_hash = h
                    _send_to_windows(data, mime)
                    continue
                last_wl_hash = h

            # Check X11 clipboard (Claude /copy)
            data, mime = _get_x11_clipboard()
            if data:
                h = hashlib.sha256(data).hexdigest()
                if h != last_x11_hash:
                    last_x11_hash = h
                    _send_to_windows(data, mime)
                    continue
                last_x11_hash = h
        except Exception:
            pass


# --- Main -------------------------------------------------------------------

if __name__ == "__main__":
    threading.Thread(target=run_receiver, daemon=True).start()
    run_sender()
