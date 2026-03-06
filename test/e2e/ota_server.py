"""Mock OTA update server for testing.

Usable as:
- pytest fixture: `ota_server` fixture in conftest.py
- standalone: python -m test.e2e.ota_server --port 8080
"""
import argparse
import hashlib
import json
import os

from aiohttp import web


class OTAMockServer:
    """Configurable mock OTA server."""

    def __init__(self, fixtures_dir=None):
        self.fixtures_dir = fixtures_dir or os.path.join(
            os.path.dirname(__file__), "fixtures"
        )
        self.app = web.Application()
        self.diagnostics_log = []
        self.request_log = []
        # Configurable responses
        self.versions_override = None
        self.error_mode = None  # None, "500", "corrupt", "timeout"

        self._setup_routes()

    def _setup_routes(self):
        self.app.router.add_get("/versions.json", self._handle_versions)
        self.app.router.add_get("/firmware/{version}.bin", self._handle_firmware)
        self.app.router.add_get("/files/{version}/{filename}", self._handle_file)
        self.app.router.add_post("/diagnostics", self._handle_diagnostics)

    async def _handle_versions(self, request):
        self.request_log.append(("GET", "/versions.json", dict(request.headers)))
        if self.error_mode == "500":
            return web.Response(status=500)
        if self.versions_override:
            return web.json_response(self.versions_override)
        manifest_path = os.path.join(self.fixtures_dir, "versions.json")
        with open(manifest_path) as f:
            manifest = json.load(f)
        # Fill in SHA-256 for firmware from fixtures/firmware/<ver>.bin
        for ver, info in manifest.get("firmware", {}).items():
            bin_path = os.path.join(self.fixtures_dir, "firmware", f"{ver}.bin")
            if os.path.exists(bin_path):
                with open(bin_path, "rb") as bf:
                    data = bf.read()
                    info["sha256"] = hashlib.sha256(data).hexdigest()
                    info["size"] = len(data)
        # Fill in SHA-256 for files from fixtures/files/<ver>/<filename>
        for ver, ver_info in manifest.get("files", {}).items():
            for change in ver_info.get("changes", []):
                file_path = os.path.join(
                    self.fixtures_dir,
                    "files",
                    ver,
                    os.path.basename(change.get("url", "")),
                )
                if os.path.exists(file_path):
                    with open(file_path, "rb") as ff:
                        data = ff.read()
                        change["sha256"] = hashlib.sha256(data).hexdigest()
        return web.json_response(manifest)

    async def _handle_firmware(self, request):
        version = request.match_info["version"]
        self.request_log.append(
            ("GET", f"/firmware/{version}.bin", dict(request.headers))
        )
        if self.error_mode == "500":
            return web.Response(status=500)
        bin_path = os.path.join(self.fixtures_dir, "firmware", f"{version}.bin")
        if not os.path.exists(bin_path):
            return web.Response(status=404)
        with open(bin_path, "rb") as f:
            data = f.read()
        if self.error_mode == "corrupt":
            data = data[: len(data) // 2]
        return web.Response(body=data, content_type="application/octet-stream")

    async def _handle_file(self, request):
        version = request.match_info["version"]
        filename = request.match_info["filename"]
        self.request_log.append(
            ("GET", f"/files/{version}/{filename}", dict(request.headers))
        )
        if self.error_mode == "500":
            return web.Response(status=500)
        file_path = os.path.join(
            self.fixtures_dir, "files", version, os.path.basename(filename)
        )
        if not os.path.exists(file_path):
            return web.Response(status=404, text="File not found")
        with open(file_path, "rb") as f:
            data = f.read()
        return web.Response(body=data, content_type="application/octet-stream")

    async def _handle_diagnostics(self, request):
        self.request_log.append(("POST", "/diagnostics", dict(request.headers)))
        if self.error_mode == "500":
            return web.Response(status=500)
        body = await request.json()
        self.diagnostics_log.append(body)
        return web.json_response({"status": "ok"})

    def reset(self):
        """Reset all state."""
        self.diagnostics_log.clear()
        self.request_log.clear()
        self.versions_override = None
        self.error_mode = None


async def start_server(host="127.0.0.1", port=0):
    """Start server, return (mock, runner, url)."""
    mock = OTAMockServer()
    runner = web.AppRunner(mock.app)
    await runner.setup()
    site = web.TCPSite(runner, host, port)
    await site.start()
    actual_port = site._server.sockets[0].getsockname()[1]
    return mock, runner, f"http://{host}:{actual_port}"


def main():
    parser = argparse.ArgumentParser(description="Mock OTA update server")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--host", default="0.0.0.0")
    args = parser.parse_args()

    mock = OTAMockServer()
    print(f"OTA mock server starting on http://{args.host}:{args.port}")
    web.run_app(mock.app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
