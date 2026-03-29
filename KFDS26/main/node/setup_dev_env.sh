#!/usr/bin/env bash
# KFDS26 Main Node — Development Environment Setup
# Target: openSUSE Tumbleweed with ESP-IDF v5.4 for ESP32-S3
set -euo pipefail

ESP_IDF_VERSION="v5.4"
ESP_IDF_DIR="$HOME/esp/esp-idf"
TARGET_CHIP="esp32s3"

echo "=== KFDS26 Dev Environment Setup ==="
echo "Distro:          openSUSE Tumbleweed"
echo "ESP-IDF version: $ESP_IDF_VERSION"
echo "Target chip:     $TARGET_CHIP"
echo ""

# ---- 1. System prerequisites ----
echo "[1/6] Installing system prerequisites..."
sudo zypper refresh
sudo zypper install -y \
    git wget curl \
    cmake ninja \
    python3 python3-pip python3-virtualenv \
    libffi-devel libopenssl-devel \
    dfu-util \
    libusb-1_0-devel \
    gcc gcc-c++ \
    flex bison gperf \
    ccache \
    picocom \
    python3-devel

echo "  System packages installed."

# ---- 2. Clone ESP-IDF ----
echo "[2/6] Setting up ESP-IDF..."
if [ -d "$ESP_IDF_DIR" ]; then
    echo "  ESP-IDF directory already exists at $ESP_IDF_DIR"
    cd "$ESP_IDF_DIR"
    CURRENT_TAG=$(git describe --tags --exact-match 2>/dev/null || echo "unknown")
    if [ "$CURRENT_TAG" != "$ESP_IDF_VERSION" ]; then
        echo "  WARNING: Current tag is '$CURRENT_TAG', expected '$ESP_IDF_VERSION'"
        echo "  Run 'cd $ESP_IDF_DIR && git checkout $ESP_IDF_VERSION && git submodule update --init --recursive' to switch."
    else
        echo "  ESP-IDF $ESP_IDF_VERSION already checked out."
    fi
else
    mkdir -p "$(dirname "$ESP_IDF_DIR")"
    git clone --branch "$ESP_IDF_VERSION" --recursive https://github.com/espressif/esp-idf.git "$ESP_IDF_DIR"
    echo "  ESP-IDF cloned to $ESP_IDF_DIR"
fi

# ---- 3. Install ESP-IDF toolchain for ESP32-S3 ----
echo "[3/6] Installing ESP-IDF toolchain for $TARGET_CHIP..."
cd "$ESP_IDF_DIR"
./install.sh "$TARGET_CHIP"
echo "  Toolchain installed."

# ---- 4. Create convenience source script ----
EXPORT_SCRIPT="$HOME/esp/source_idf.sh"
echo "[4/6] Creating convenience script: $EXPORT_SCRIPT"
cat > "$EXPORT_SCRIPT" << 'INNEREOF'
#!/usr/bin/env bash
# Source this file to set up ESP-IDF environment:
#   source ~/esp/source_idf.sh
export IDF_PATH="$HOME/esp/esp-idf"
source "$IDF_PATH/export.sh"
INNEREOF
chmod +x "$EXPORT_SCRIPT"
echo "  Created. Usage: source $EXPORT_SCRIPT"

# ---- 5. Add user to dialout group for serial port access ----
echo "[5/6] Adding user to dialout group for serial port access..."
if groups | grep -q dialout; then
    echo "  Already in dialout group."
else
    sudo usermod -aG dialout "$USER"
    echo "  Added to dialout group. Log out and back in for it to take effect."
fi

# ---- 6. Print CLion + terminal configuration ----
echo ""
echo "[6/6] Configuration Instructions"
echo "========================================"
echo ""
echo "A) TERMINAL BUILD (recommended for first build):"
echo "   source ~/esp/source_idf.sh"
echo "   cd <project-dir>/KFDS26/main/node"
echo "   idf.py set-target $TARGET_CHIP"
echo "   idf.py build"
echo "   idf.py -p /dev/ttyUSB0 flash monitor"
echo ""
echo "B) CLION SETUP:"
echo ""
echo "   1. Open KFDS26/main/node as project in CLion"
echo ""
echo "   2. Go to: Settings > Build, Execution, Deployment > Toolchains"
echo "      - Click 'Add environment' next to the Name field, then 'From file'"
echo "      - Select: $ESP_IDF_DIR/export.sh"
echo "      - If using a Python venv, you may need a wrapper script that sources both"
echo "        the venv activate and export.sh. We created: $EXPORT_SCRIPT"
echo "      - Save the toolchain. The project will reload automatically."
echo ""
echo "   3. If CMake loading fails, go to: File > Reload CMake Project"
echo ""
echo "   4. Build targets will appear automatically:"
echo "      - 'app'     — build the firmware"
echo "      - 'flash'   — flash to device"
echo "      - 'monitor' — serial monitor"
echo ""
echo "   5. Set the ESP serial port (if flash target fails):"
echo "      Settings > Build > CMake > Environment:"
echo "        ESPPORT=/dev/ttyUSB0   (or /dev/ttyACM0 for USB-CDC)"
echo ""
echo "   6. CLion Terminal:"
echo "      Settings > Tools > Terminal > Shell path:"
echo "        /bin/bash --rcfile ~/esp/source_idf.sh"
echo "      This ensures idf.py works in CLion's integrated terminal."
echo ""
echo "========================================"
echo "Setup complete!"
