"""HTTP client for board OTA and device API endpoints."""

import json
import logging
import urllib.request

_log = logging.getLogger("board.http")


class BoardHTTPClient:
    """Wraps the board's HTTP API for OTA operations."""

    def __init__(self, base_url: str, timeout: float = 30.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    def _request(self, method: str, path: str, body: dict | None = None) -> dict:
        url = self.base_url + path
        data = None
        headers = {}
        if body is not None:
            data = json.dumps(body).encode()
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            return json.loads(resp.read())

    def get_device_status(self) -> dict:
        """GET /api/device/status"""
        result = self._request("GET", "/api/device/status")
        _log.info("device status: %s", result)
        return result

    def get_ota_config(self) -> dict:
        """GET /api/ota/config"""
        result = self._request("GET", "/api/ota/config")
        _log.info("OTA config: %s", result)
        return result

    def put_ota_config(self, cfg: dict) -> dict:
        """PUT /api/ota/config"""
        result = self._request("PUT", "/api/ota/config", body=cfg)
        _log.info("OTA config updated: %s", result)
        return result

    def ota_check(self) -> dict:
        """POST /api/ota/check"""
        result = self._request("POST", "/api/ota/check")
        _log.info("OTA check: %s", result)
        return result

    def ota_update_firmware(self, manifest: dict, version: str) -> dict:
        """POST /api/ota/update/firmware"""
        result = self._request(
            "POST",
            "/api/ota/update/firmware",
            body={"manifest": manifest, "version": version},
        )
        _log.info("OTA firmware update: %s", result)
        return result

    def ota_update_files(self, manifest: dict, version: str) -> dict:
        """POST /api/ota/update/files"""
        result = self._request(
            "POST",
            "/api/ota/update/files",
            body={"manifest": manifest, "version": version},
        )
        _log.info("OTA file update: %s", result)
        return result

    def get_device_log(self, name: str) -> str:
        """GET /api/device/log?file={name}

        name: 'boot', 'boot.prev', 'check', 'update'
        """
        result = self._request("GET", f"/api/device/log?file={name}")
        log_content = result.get("log", "")
        _log.info("device log '%s': %d chars", name, len(log_content))
        return log_content
