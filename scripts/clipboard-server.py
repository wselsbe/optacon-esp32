#!/usr/bin/env python3
"""Tiny HTTP server that receives clipboard data and sets it via wl-copy.

Listens on 127.0.0.1:8224 (SSH-tunnel only). The Windows client POSTs
text or image data, and this server pipes it into wl-copy with the
correct MIME type.
"""

import subprocess
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer

LISTEN = "127.0.0.1"
PORT = 8224


class ClipHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        content_type = self.headers.get("Content-Type", "text/plain")
        length = int(self.headers.get("Content-Length", 0))
        data = self.rfile.read(length)
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

    def log_message(self, fmt, *args):
        # Quiet — only log errors
        pass


if __name__ == "__main__":
    print(f"Clipboard server listening on {LISTEN}:{PORT}")
    HTTPServer((LISTEN, PORT), ClipHandler).serve_forever()
