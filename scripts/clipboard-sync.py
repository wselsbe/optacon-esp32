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
            proc = subprocess.run(
                ["wl-copy", "--type", content_type],
                input=data,
                timeout=5,
            )
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


def _get_clipboard():
    """Read the Wayland clipboard. Returns (data, mime_type) or (None, None)."""
    # Check available types
    try:
        types = subprocess.run(
            ["wl-paste", "--list-types"],
            capture_output=True,
            timeout=2,
        ).stdout.decode().strip().splitlines()
    except Exception:
        return None, None

    # Prefer image if available
    for t in types:
        if t.startswith("image/"):
            try:
                data = subprocess.run(
                    ["wl-paste", "--type", t, "--no-newline"],
                    capture_output=True,
                    timeout=2,
                ).stdout
                if data:
                    return data, t
            except Exception:
                pass
            break

    # Fall back to text
    try:
        data = subprocess.run(
            ["wl-paste", "--no-newline"],
            capture_output=True,
            timeout=2,
        ).stdout
        if data:
            return data, "text/plain;charset=utf-8"
    except Exception:
        pass
    return None, None


def run_sender():
    last_hash = None
    print(f"Sender polling clipboard → 127.0.0.1:{SEND_PORT}")
    while True:
        time.sleep(POLL_INTERVAL)
        try:
            data, mime = _get_clipboard()
            if data is None:
                continue
            h = hashlib.sha256(data).hexdigest()
            # Skip if unchanged or if this is something we just received from Windows
            if h == last_hash:
                continue
            with _lock:
                if h == _last_received_hash:
                    last_hash = h
                    continue
            last_hash = h
            req = urllib.request.Request(
                SEND_URL,
                data=data,
                headers={"Content-Type": mime},
                method="POST",
            )
            urllib.request.urlopen(req, timeout=3)
        except Exception:
            pass


# --- Main -------------------------------------------------------------------

if __name__ == "__main__":
    threading.Thread(target=run_receiver, daemon=True).start()
    run_sender()
