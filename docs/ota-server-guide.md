# OTA Update Server Guide

How to host firmware and file updates for the Optacon ESP32.

## What the device expects

The device fetches `{update_url}/versions.json`, then downloads files relative to that same base URL. All requests are plain HTTP GET — no auth, no API, just static files.

## versions.json format

```json
{
  "latest_firmware": "0.3.0",
  "latest_files": "0.2.0",
  "firmware": {
    "0.3.0": {
      "url": "firmware/0.3.0/micropython.bin",
      "size": 1375216,
      "sha256": "47a743d43e1cb25400c8b06482c969fbf3b6d6fc1cae2105490a3db0ed0947ce"
    },
    "0.2.0": {
      "url": "firmware/0.2.0/micropython.bin",
      "size": 1370000,
      "sha256": "abc123..."
    }
  },
  "files": {
    "0.2.0": {
      "changes": [
        {
          "path": "web_server.py",
          "url": "files/0.2.0/web_server.py",
          "sha256": "def456..."
        },
        {
          "path": "web/index.html",
          "url": "files/0.2.0/web/index.html",
          "sha256": "789abc..."
        }
      ]
    }
  }
}
```

### Fields

| Field | Description |
|-------|-------------|
| `latest_firmware` | Semver string. Device compares against its current version. |
| `latest_files` | Semver string for filesystem Python/web updates. |
| `firmware.<ver>.url` | Path relative to `update_url`. Device GETs this. |
| `firmware.<ver>.size` | Exact byte count. Must match or download is rejected. |
| `firmware.<ver>.sha256` | Hex-encoded SHA-256. Verified before setting boot partition. |
| `files.<ver>.changes[].path` | Destination on device filesystem (e.g. `web_server.py`, `web/index.html`). |
| `files.<ver>.changes[].url` | Download path relative to `update_url`. |
| `files.<ver>.changes[].sha256` | Hex-encoded SHA-256 of the file. |

### Version comparison

Semver-style: `0.3.0 > 0.2.0 > 0.1.0`. The device only offers updates when the server version is strictly greater than the installed version.

## Directory structure

```
ota-root/
├── versions.json
├── firmware/
│   ├── 0.1.0/
│   │   └── micropython.bin
│   ├── 0.2.0/
│   │   └── micropython.bin
│   └── 0.3.0/
│       └── micropython.bin
└── files/
    ├── 0.1.0/
    │   ├── web_server.py
    │   ├── wifi.py
    │   └── web/
    │       └── index.html
    └── 0.2.0/
        └── web_server.py
```

The URL paths in `versions.json` are relative — you can use any directory layout as long as the URLs match.

## Publishing a new version

### Firmware update

1. Build firmware (`./scripts/build.sh`).
2. Compute the SHA-256:
   ```bash
   sha256sum build/micropython.bin
   ```
3. Copy the binary:
   ```bash
   mkdir -p ota-root/firmware/0.3.0
   cp build/micropython.bin ota-root/firmware/0.3.0/
   ```
4. Update `versions.json`:
   - Set `latest_firmware` to `"0.3.0"`
   - Add the entry under `firmware` with `url`, `size`, and `sha256`
5. **Update `boot_cfg.py`** — set `FIRMWARE_VERSION = "0.3.0"` before building, so the device reports the correct version after updating.

### File update

1. Compute SHA-256 for each changed file:
   ```bash
   sha256sum python/web_server.py python/wifi.py web/index.html
   ```
2. Copy the files:
   ```bash
   mkdir -p ota-root/files/0.2.0/web
   cp python/web_server.py python/wifi.py ota-root/files/0.2.0/
   cp web/index.html ota-root/files/0.2.0/web/
   ```
3. Update `versions.json`:
   - Set `latest_files` to `"0.2.0"`
   - Add a `changes` array listing each file with `path`, `url`, and `sha256`

### Helper script

`scripts/ota-publish.sh` can automate the above:

```bash
#!/bin/bash
# Usage: ./scripts/ota-publish.sh <firmware|files> <version> <ota-root>
set -e

TYPE="$1"
VERSION="$2"
OTA_ROOT="$3"

if [ "$TYPE" = "firmware" ]; then
    DIR="${OTA_ROOT}/firmware/${VERSION}"
    mkdir -p "$DIR"
    cp build/micropython.bin "$DIR/"
    SIZE=$(stat -c%s build/micropython.bin)
    SHA=$(sha256sum build/micropython.bin | cut -d' ' -f1)
    echo "Add to versions.json → firmware.${VERSION}:"
    echo "  url: firmware/${VERSION}/micropython.bin"
    echo "  size: ${SIZE}"
    echo "  sha256: ${SHA}"

elif [ "$TYPE" = "files" ]; then
    DIR="${OTA_ROOT}/files/${VERSION}"
    shift 3  # remaining args are file paths
    for FILE in "$@"; do
        DEST="${DIR}/${FILE}"
        mkdir -p "$(dirname "$DEST")"
        cp "$FILE" "$DEST"
        SHA=$(sha256sum "$FILE" | cut -d' ' -f1)
        echo "  path: ${FILE}  url: files/${VERSION}/${FILE}  sha256: ${SHA}"
    done
fi
```

## Hosting options

### Option A: nginx (self-hosted)

Serve the `ota-root/` directory as static files:

```nginx
server {
    listen 80;
    server_name ota.example.com;

    root /var/www/optacon-ota;
    autoindex off;

    # CORS for browser-based tools (optional)
    add_header Access-Control-Allow-Origin *;

    location / {
        try_files $uri =404;
    }
}
```

Set the device's update URL to `http://ota.example.com`.

**Note:** The ESP32 OTA client supports HTTPS but it adds ~50 KB RAM overhead and requires a CA bundle. HTTP is fine for local/private networks.

### Option B: GitHub Pages (linked to releases)

Use the repo's GitHub Pages site to serve `versions.json` and binaries.

**Setup:**

1. Enable GitHub Pages on the repo (Settings → Pages → source: GitHub Actions).
2. Create a `gh-pages` branch or `docs/` folder with the OTA directory structure.
3. Set the device's update URL to `https://wselsbe.github.io/optacon-esp32`.

**Directory layout** (in the gh-pages branch or docs folder):

```
versions.json
firmware/
  0.3.0/
    micropython.bin
files/
  0.2.0/
    web_server.py
```

**Limitations:**
- GitHub Pages files max 100 MB (firmware is ~1.4 MB, so fine).
- Site max 1 GB total. Keep only the latest few firmware versions.
- HTTPS only — the ESP32 needs a CA bundle or `use_ssl=True` support.

### Option C: GitHub Releases (automated via CI)

Attach `micropython.bin` and filesystem files to GitHub Releases, then have CI auto-generate `versions.json`.

**Workflow addition** (`.github/workflows/release.yml`):

```yaml
name: Release

on:
  push:
    tags: ['v*']

jobs:
  build-and-release:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v3

      - name: Build Docker image
        uses: docker/build-push-action@v6
        with:
          context: .
          load: true
          tags: optacon-fw:release
          cache-from: type=gha
          cache-to: type=gha,mode=max

      - name: Build firmware
        run: |
          docker run --rm \
            -v "${{ github.workspace }}:/workspace" \
            optacon-fw:release \
            bash /workspace/scripts/build.sh

      - name: Compute SHA-256
        id: sha
        run: echo "sha256=$(sha256sum build/micropython.bin | cut -d' ' -f1)" >> "$GITHUB_OUTPUT"

      - name: Create release
        uses: softprops/action-gh-release@v2
        with:
          files: |
            build/micropython.bin
            build/bootloader.bin
            build/partition-table.bin
          body: |
            **Firmware SHA-256:** `${{ steps.sha.outputs.sha256 }}`
            **Size:** $(stat -c%s build/micropython.bin) bytes

      - name: Update versions.json on gh-pages
        run: |
          VERSION="${GITHUB_REF_NAME#v}"  # v0.3.0 → 0.3.0
          SIZE=$(stat -c%s build/micropython.bin)
          SHA="${{ steps.sha.outputs.sha256 }}"

          git fetch origin gh-pages
          git worktree add gh-pages origin/gh-pages

          # Update versions.json with jq
          cd gh-pages
          mkdir -p "firmware/${VERSION}"
          cp ../build/micropython.bin "firmware/${VERSION}/"

          # Update manifest
          jq --arg v "$VERSION" \
             --arg sha "$SHA" \
             --argjson size "$SIZE" \
             '.latest_firmware = $v |
              .firmware[$v] = {
                url: ("firmware/" + $v + "/micropython.bin"),
                size: $size,
                sha256: $sha
              }' versions.json > versions.json.tmp
          mv versions.json.tmp versions.json

          git add .
          git commit -m "release: firmware ${VERSION}"
          git push origin gh-pages
```

**Usage:** Tag a release → CI builds → binary attached to release → `versions.json` updated on gh-pages → devices pick it up on next check.

```bash
# Bump version in boot_cfg.py first, then:
git tag v0.3.0
git push origin v0.3.0
```

### Recommended approach

Start with **nginx on a local machine** for development/testing. Move to **GitHub Pages + CI releases** for production — it's free, automated, and doesn't require infrastructure.

## Device configuration

Set the update URL on the device via:

- **Web UI:** Go to `/update`, enter the URL in Settings, click SAVE.
- **WebSocket command:** `{"cmd": "wifi_config", ...}` (or use the exec command).
- **Directly on filesystem:** Edit `/ota_config.json`:
  ```json
  {
    "update_url": "http://192.168.1.100:8080",
    "auto_check": true,
    "firmware_version": "0.1.0",
    "files_version": "0.1.0"
  }
  ```

## Constraints

- **Firmware binary must fit in 1.5 MB** (OTA partition size). Current build is ~1.37 MB.
- **No authentication** — the OTA client does plain HTTP GET. Restrict access via network segmentation or firewall rules if needed.
- **SHA-256 is mandatory in practice** — the device verifies it and rejects mismatches. Never publish without correct hashes.
- **Keep old versions** — if a device has version 0.1.0 and you remove 0.2.0 from the server while publishing 0.3.0, that's fine. The device only downloads `latest_firmware`, not intermediate versions. But keeping old binaries is useful for debugging.
