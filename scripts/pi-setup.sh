#!/usr/bin/env bash
# Raspberry Pi 4 setup for Optacon firmware development
# Run once after first boot: bash scripts/pi-setup.sh
# Idempotent — safe to re-run.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ESP_IDF_VER="v5.5.1"
MPY_VER="v1.27.0"
NODE_MAJOR=20

green()  { printf '\033[1;32m%s\033[0m\n' "$*"; }
yellow() { printf '\033[1;33m%s\033[0m\n' "$*"; }

# ── Step 1: System packages ──────────────────────────────────────────────────

green "Step 1: System packages"
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
  git cmake ninja-build ccache libffi-dev libssl-dev dfu-util \
  libusb-1.0-0-dev libudev-dev \
  python3 python3-pip python3-venv \
  sigrok sigrok-cli \
  curl wget

# ── Step 2: Node.js 20 LTS ───────────────────────────────────────────────────

green "Step 2: Node.js ${NODE_MAJOR} LTS"
if command -v node &>/dev/null && node --version | grep -q "^v${NODE_MAJOR}\."; then
  yellow "  Node.js $(node --version) already installed, skipping"
else
  curl -fsSL "https://deb.nodesource.com/setup_${NODE_MAJOR}.x" | sudo -E bash -
  sudo apt install -y nodejs
fi

# ── Step 3: uv (Python package runner) ───────────────────────────────────────

green "Step 3: uv"
if command -v uv &>/dev/null; then
  yellow "  uv already installed ($(uv --version)), skipping"
else
  curl -LsSf https://astral.sh/uv/install.sh | sh
  # Make uv available for the rest of this script
  export PATH="$HOME/.local/bin:$PATH"
fi

# ── Step 4: ESP-IDF v5.5.1 ───────────────────────────────────────────────────

green "Step 4: ESP-IDF ${ESP_IDF_VER}"
if [ -d "$HOME/esp/${ESP_IDF_VER}" ]; then
  yellow "  ~/esp/${ESP_IDF_VER} exists, skipping clone"
else
  mkdir -p ~/esp
  git clone --depth 1 --branch "${ESP_IDF_VER}" \
    https://github.com/espressif/esp-idf.git "$HOME/esp/${ESP_IDF_VER}"
  cd "$HOME/esp/${ESP_IDF_VER}"
  git submodule update --init --recursive --depth 1
fi

# Always run install.sh — it's idempotent and ensures tools are present
cd "$HOME/esp/${ESP_IDF_VER}"
./install.sh esp32s3

# ── Step 5: MicroPython v1.27.0 ──────────────────────────────────────────────

green "Step 5: MicroPython ${MPY_VER}"
if [ -d "$HOME/micropython" ]; then
  yellow "  ~/micropython exists, skipping clone"
else
  cd ~
  git clone --depth 1 --branch "${MPY_VER}" \
    https://github.com/micropython/micropython.git
  cd ~/micropython
  git submodule update --init --depth 1 lib/berkeley-db-1.xx
  git submodule update --init --depth 1 lib/micropython-lib
fi

# Build mpy-cross (idempotent — make skips if up to date)
green "  Building mpy-cross"
source "$HOME/esp/${ESP_IDF_VER}/export.sh"
make -C ~/micropython/mpy-cross

# ── Step 6: Clone optacon-firmware ────────────────────────────────────────────

green "Step 6: optacon-firmware"
if [ -d "$HOME/projects/optacon-firmware" ]; then
  yellow "  ~/projects/optacon-firmware exists, skipping clone"
else
  mkdir -p ~/projects
  cd ~/projects
  git clone https://github.com/wselsbe/optacon-esp32.git optacon-firmware
fi

# ── Step 7: Python tools ─────────────────────────────────────────────────────

green "Step 7: Python tools (mpremote, esptool, ruff)"
pip install --break-system-packages mpremote esptool ruff

# ── Step 8: Claude Code ──────────────────────────────────────────────────────

green "Step 8: Claude Code"
if command -v claude &>/dev/null; then
  yellow "  Claude Code already installed ($(claude --version 2>/dev/null || echo 'unknown')), skipping"
else
  sudo npm install -g @anthropic-ai/claude-code
fi

# ── Step 9: USB udev rules ───────────────────────────────────────────────────

green "Step 9: udev rules"
RULES_SRC="${SCRIPT_DIR}/99-optacon.rules"
RULES_DST="/etc/udev/rules.d/99-optacon.rules"
if [ -f "$RULES_SRC" ]; then
  sudo cp "$RULES_SRC" "$RULES_DST"
  sudo udevadm control --reload-rules && sudo udevadm trigger
  green "  Installed ${RULES_DST}"
else
  yellow "  WARNING: ${RULES_SRC} not found, skipping udev rules"
fi
sudo usermod -aG dialout pi 2>/dev/null || true

# ── Step 10: Claude Code MCP servers ─────────────────────────────────────────

green "Step 10: Claude Code MCP servers"
mkdir -p ~/.claude
cat > ~/.claude/settings.json << 'SETTINGS_EOF'
{
  "mcpServers": {
    "micropython": {
      "command": "uvx",
      "args": ["mpremote-mcp==0.1.1"],
      "env": {
        "MPY_VID": "0x303A"
      }
    },
    "siglent-sds": {
      "command": "npx",
      "args": ["-y", "siglent-sds-mcp"],
      "env": {
        "SIGLENT_IP": "192.168.30.236"
      }
    },
    "siglent-spd": {
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
      "command": "uvx",
      "args": ["sigrok-logicanalyzer-mcp"]
    }
  }
}
SETTINGS_EOF

# ── Step 11: Shell environment ────────────────────────────────────────────────

green "Step 11: Shell environment (~/.bashrc)"
MARKER="# --- Optacon development ---"
if grep -qF "$MARKER" ~/.bashrc 2>/dev/null; then
  yellow "  Bashrc block already present, skipping"
else
  cat >> ~/.bashrc << 'BASHRC_EOF'

# --- Optacon development ---

# ESP-IDF
alias idf-init='source ~/esp/v5.5.1/export.sh'

# MicroPython
export MICROPYTHON_DIR=~/micropython

# Optacon
alias optacon-build='cd ~/projects/optacon-firmware && idf-init && ./scripts/build.sh'
alias optacon-flash='./scripts/flash.sh /dev/esp32s3-boot'
BASHRC_EOF
  green "  Appended aliases to ~/.bashrc"
fi

# ── Done ──────────────────────────────────────────────────────────────────────

green ""
green "Setup complete! Next steps:"
green "  1. Reboot:  sudo reboot"
green "  2. Verify:  source ~/esp/v5.5.1/export.sh && xtensa-esp32s3-elf-gcc --version"
green "  3. Build:   cd ~/projects/optacon-firmware && idf-init && ./scripts/build.sh"
green "  4. Claude:  claude --version"
