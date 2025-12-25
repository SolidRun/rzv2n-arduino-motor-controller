#!/bin/bash
#
# Installation Script for Bluetooth-Arduino Bridge (Python Version)
# Run as root: sudo ./install.sh
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="/opt/bt_arduino_bridge"
SERVICE_FILE="/etc/systemd/system/bt-arduino-bridge.service"
LOG_FILE="/var/log/bt_arduino_bridge.log"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
echo_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
echo_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo_error "Please run as root (sudo ./install.sh)"
    exit 1
fi

echo "=========================================="
echo " Bluetooth-Arduino Bridge Installation"
echo "=========================================="
echo

# Detect package manager
detect_pkg_manager() {
    if command -v apt-get &>/dev/null; then
        echo "apt"
    elif command -v dnf &>/dev/null; then
        echo "dnf"
    elif command -v yum &>/dev/null; then
        echo "yum"
    elif command -v pacman &>/dev/null; then
        echo "pacman"
    elif command -v zypper &>/dev/null; then
        echo "zypper"
    elif command -v apk &>/dev/null; then
        echo "apk"
    else
        echo "unknown"
    fi
}

PKG_MANAGER=$(detect_pkg_manager)
echo_info "Detected package manager: $PKG_MANAGER"

# Install dependencies based on package manager
echo_info "Installing dependencies..."

case "$PKG_MANAGER" in
    apt)
        apt-get update -qq
        apt-get install -y bluetooth bluez bluez-tools rfkill python3 python3-serial coreutils || true
        ;;
    dnf|yum)
        $PKG_MANAGER install -y bluez bluez-tools rfkill python3 python3-pyserial coreutils || true
        ;;
    pacman)
        pacman -Sy --noconfirm bluez bluez-utils rfkill python python-pyserial coreutils || true
        ;;
    zypper)
        zypper install -y bluez bluez-tools rfkill python3 python3-pyserial coreutils || true
        ;;
    apk)
        apk add bluez bluez-deprecated rfkill python3 py3-serial coreutils || true
        ;;
    *)
        echo_warn "Unknown package manager. Please install manually:"
        echo_warn "  - bluez (Bluetooth stack)"
        echo_warn "  - bluez-tools (rfcomm, hciconfig)"
        echo_warn "  - rfkill"
        echo_warn "  - python3"
        echo_warn "  - python3-pyserial"
        echo_warn "  - coreutils"
        ;;
esac

echo_info "Dependencies installed"

# Create installation directory
echo_info "Creating installation directory..."
mkdir -p "$INSTALL_DIR"

# Copy Python scripts
echo_info "Installing Python bridge scripts..."
cp "$SCRIPT_DIR/bt_arduino_bridge.py" "$INSTALL_DIR/"
cp "$SCRIPT_DIR/handle_connection.py" "$INSTALL_DIR/"
chmod +x "$INSTALL_DIR/bt_arduino_bridge.py"
chmod +x "$INSTALL_DIR/handle_connection.py"

# Also keep bash scripts as backup
cp "$SCRIPT_DIR/bt_arduino_bridge.sh" "$INSTALL_DIR/" 2>/dev/null || true
cp "$SCRIPT_DIR/handle_connection.sh" "$INSTALL_DIR/" 2>/dev/null || true
chmod +x "$INSTALL_DIR/bt_arduino_bridge.sh" 2>/dev/null || true
chmod +x "$INSTALL_DIR/handle_connection.sh" 2>/dev/null || true

# Create/update systemd service file for Python script
echo_info "Creating systemd service file..."
cat > "$SERVICE_FILE" << EOF
[Unit]
Description=Bluetooth to Arduino Bridge Service (Python)
After=bluetooth.target
Wants=bluetooth.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 $INSTALL_DIR/bt_arduino_bridge.py
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal
User=root

# Bluetooth permissions
CapabilityBoundingSet=CAP_NET_ADMIN CAP_NET_RAW CAP_NET_BIND_SERVICE
AmbientCapabilities=CAP_NET_ADMIN CAP_NET_RAW CAP_NET_BIND_SERVICE

[Install]
WantedBy=multi-user.target
EOF

chmod 644 "$SERVICE_FILE"

# Create log file
echo_info "Setting up logging..."
touch "$LOG_FILE"
chmod 644 "$LOG_FILE"

# Enable and start Bluetooth service
echo_info "Ensuring Bluetooth service is running..."
if command -v systemctl &>/dev/null; then
    systemctl enable bluetooth.service 2>/dev/null || true
    systemctl start bluetooth.service 2>/dev/null || true
fi

# Wait for Bluetooth service to be ready
sleep 2

# Set device name
echo_info "Setting Bluetooth device name to Arduino_BT_Bridge..."
hciconfig hci0 name 'Arduino_BT_Bridge' 2>/dev/null || true
echo "PRETTY_HOSTNAME=Arduino_BT_Bridge" > /etc/machine-info 2>/dev/null || true

# Configure Bluetooth using bluetoothctl
echo_info "Configuring Bluetooth (discoverable, pairable)..."
bluetoothctl << EOF
system-alias Arduino_BT_Bridge
power on
discoverable on
pairable on
agent on
exit
EOF

# Reload systemd
echo_info "Reloading systemd..."
systemctl daemon-reload

# Enable service
echo_info "Enabling service for boot..."
systemctl enable bt-arduino-bridge.service

# Start service
echo_info "Starting service..."
systemctl start bt-arduino-bridge.service

# Wait a moment and check status
sleep 3
if systemctl is-active --quiet bt-arduino-bridge.service; then
    echo_info "Service is running!"
else
    echo_warn "Service may not have started correctly"
    echo_warn "Check logs: journalctl -u bt-arduino-bridge.service -f"
fi

echo
echo "=========================================="
echo " Installation Complete!"
echo "=========================================="
echo
echo "Commands:"
echo "  Start:   systemctl start bt-arduino-bridge"
echo "  Stop:    systemctl stop bt-arduino-bridge"
echo "  Status:  systemctl status bt-arduino-bridge"
echo "  Restart: systemctl restart bt-arduino-bridge"
echo
echo "Logs:"
echo "  Service: journalctl -u bt-arduino-bridge.service -f"
echo "  Debug:   tail -f /tmp/bt_bridge_debug.log"
echo "  Commands: tail -f /tmp/bt_bridge_commands.log"
echo
echo "Bluetooth device name: Arduino_BT_Bridge"
echo "Connect from phone/laptop and send commands!"
echo
