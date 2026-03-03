# Raspberry Pi 4 Setup

Headless development station for the Optacon firmware. Runs Claude Code with all MCP servers (ESP32, oscilloscope, power supply, logic analyzer), auto-starts on boot with remote control enabled.

## Hardware

- Raspberry Pi 4B (4GB+), Raspberry Pi OS 64-bit (Bookworm/Trixie)
- ESP32-S3 connected via USB
- Zeroplus LAP-C 16128 logic analyzer via USB
- Siglent SDS/SPD instruments on LAN

## Quick Start

### 1. Flash SD Card

Use **Raspberry Pi Imager** with these settings:

| Setting | Value |
|---------|-------|
| Hostname | `optacon-pi` |
| SSH | Yes (password auth) |
| Username | `pi` |
| WiFi SSID | *(your IoT network)* |
| WiFi Password | *(your password)* |
| WiFi Country | `BE` |
| Locale | `Europe/Brussels` |

### 2. First Boot

Insert SD card, power on, wait ~2 minutes, then SSH in and run the setup script:

```bash
ssh pi@optacon-pi.local
# clone the repo (or scp the scripts), then:
bash ~/projects/optacon-firmware/scripts/pi-setup.sh
sudo reboot
```

The setup script (`scripts/pi-setup.sh`) is idempotent and installs:
- ESP-IDF v5.5.1 + xtensa toolchain
- MicroPython v1.27.0 + mpy-cross
- Node.js 20 LTS, uv, Claude Code
- sigrok-cli, mpremote, esptool, ruff
- udev rules for ESP32-S3 and logic analyzer

### 3. Post-Setup (manual steps)

These are not automated by the setup script:

```bash
# Passwordless sudo
echo 'pi ALL=(ALL) NOPASSWD: ALL' | sudo tee /etc/sudoers.d/010_pi-nopasswd

# GitHub CLI
curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg
echo 'deb [arch=arm64 signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main' | sudo tee /etc/apt/sources.list.d/github-cli.list
sudo apt update && sudo apt install -y gh
gh auth login --web --git-protocol https
gh auth setup-git  # configure git to use gh as credential helper

# Claude Code login
claude  # then /login interactively

# Install plugins
claude plugin install context7 playwright claude-code-setup frontend-design superpowers pyright-lsp

# Install LSP dependencies
sudo apt install -y clangd
sudo npm install -g pyright

# Additional packages (not in setup script)
sudo apt install -y screen tmux

# Locale
sudo sed -i 's/# en_GB.UTF-8 UTF-8/en_GB.UTF-8 UTF-8/' /etc/locale.gen
sudo locale-gen
```

### 4. MCP Servers

Add via `claude mcp add` (stored in `~/.claude.json`, NOT `~/.claude/settings.json`):

```bash
claude mcp add -s user -t stdio micropython uvx -- mpremote-mcp==0.1.1
claude mcp add -s user -t stdio siglent-sds npx -- -y siglent-sds-mcp
claude mcp add -s user -t stdio siglent-spd uvx -- siglent-spd-mcp
claude mcp add -s user -t stdio logic-analyzer uvx -- sigrok-logicanalyzer-mcp
```

Then manually edit `~/.claude.json` to fix the `env` blocks (the `-e` flag adds to args, not env):

```json
"micropython": {
  "type": "stdio",
  "command": "uvx",
  "args": ["mpremote-mcp==0.1.1"],
  "env": { "MPY_VID": "0x303A" }
},
"siglent-sds": {
  "type": "stdio",
  "command": "npx",
  "args": ["-y", "siglent-sds-mcp"],
  "env": { "SIGLENT_IP": "192.168.30.236" }
},
"siglent-spd": {
  "type": "stdio",
  "command": "uvx",
  "args": ["siglent-spd-mcp"],
  "env": {
    "SPD_HOST": "192.168.30.134",
    "CH1_PERM": "readwrite",
    "CH1_MAX_VOLTAGE": "5.5",
    "CH1_MAX_CURRENT": "1"
  }
},
"logic-analyzer": {
  "type": "stdio",
  "command": "uvx",
  "args": ["sigrok-logicanalyzer-mcp"],
  "env": {}
}
```

## Claude Code Auto-Start

Claude Code runs in a tmux session via a systemd user service. It starts on boot, auto-restarts if exited, and resumes the previous conversation.

### Service: `~/.config/systemd/user/claude.service`

```ini
[Unit]
Description=Claude Code in tmux session
After=network-online.target
Wants=network-online.target

[Service]
Type=forking
ExecStart=/usr/bin/tmux new-session -d -s claude /home/pi/start-claude.sh
ExecStop=/usr/bin/tmux kill-session -t claude
Restart=always
RestartSec=10

[Install]
WantedBy=default.target
```

### Launcher: `~/start-claude.sh`

```bash
#!/bin/bash
export PATH="$HOME/.local/bin:/usr/local/bin:$PATH"
export COLORTERM=truecolor
export LANG=en_GB.UTF-8
export TERM=tmux-256color
export MICROPYTHON_DIR=~/micropython
source ~/esp/v5.5.1/export.sh >/dev/null 2>&1
cd ~/projects/optacon-firmware
exec claude --dangerously-skip-permissions --continue
```

### Enable

```bash
systemctl --user daemon-reload
systemctl --user enable claude.service
sudo loginctl enable-linger pi   # starts without login
systemctl --user start claude.service
```

### tmux config: `~/.tmux.conf`

```
set -g default-terminal "tmux-256color"
set -ag terminal-overrides ",xterm-256color:RGB"
set -g history-limit 10000
set -g mouse on
```

## Connecting

### SSH + tmux

```bash
ssh -t optacon-pi "tmux attach -t claude"
# Detach: Ctrl+B, D
```

Or use `scripts/optacon-pi-claude.cmd` on Windows.

### Remote Control (phone/browser)

Claude Code has remote control enabled by default. Open `claude.ai/code` to connect from any device.

### Playwright Remote Debugging

To see the browser on your laptop while Claude controls it from the Pi:

1. Start Chrome on your laptop:
   ```
   scripts/optacon-pi-chrome-remote-debug.cmd
   ```
   Or manually:
   ```
   chrome.exe --remote-debugging-port=9222 --user-data-dir="%TEMP%\chrome-debug"
   ```

2. Windows Firewall (admin terminal, one-time):
   ```
   netsh advfirewall firewall add rule name="Chrome Remote Debugging" dir=in action=allow protocol=TCP localport=9222 remoteip=192.168.0.0/16
   ```

3. The `playwright-remote-thinkpad` MCP server on the Pi connects to `http://Thinkpad:9222`.

## Directory Layout on Pi

```
~/esp/v5.5.1/              ESP-IDF
~/micropython/             MicroPython source + mpy-cross
~/projects/optacon-firmware/   This repo
~/start-claude.sh          Claude launcher script
~/.claude.json             MCP servers + Claude state
~/.claude/settings.json    Claude project settings
~/.config/systemd/user/claude.service
```

## SSH Config (Windows)

In `C:\Users\Wannes\.ssh\config`:

```
Host optacon-pi optacon-pi.local
    HostName 192.168.30.192
    User pi
    IdentityFile ~/.ssh/optacon-pi
```

## Verification

```bash
source ~/esp/v5.5.1/export.sh && xtensa-esp32s3-elf-gcc --version
~/micropython/mpy-cross/build/mpy-cross --version
claude --version
gh auth status
sigrok-cli --scan          # with logic analyzer plugged in
ls -la /dev/esp32s3        # with ESP32-S3 plugged in
```
