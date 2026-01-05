#!/bin/bash
#
# Installation Script for Bluetooth-Arduino Bridge (Python Version)
# Run as root: sudo ./install.sh
#
# Features:
#   - D-Bus agent for instant pairing confirmation
#   - RFCOMM listen for stable SPP connections
#   - Auto-reconnect on connection loss
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="/opt/bt_arduino_bridge"
SERVICE_FILE="/etc/systemd/system/bt-arduino-bridge.service"
LOG_FILE="/var/log/bt_arduino_bridge.log"
BT_NAME="Arduino_BT_Bridge"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
echo_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
echo_error() { echo -e "${RED}[ERROR]${NC} $1"; }
echo_step() { echo -e "${CYAN}[STEP]${NC} $1"; }

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo_error "Please run as root (sudo ./install.sh)"
    exit 1
fi

echo "=============================================="
echo " Bluetooth-Arduino Bridge Installation"
echo " with D-Bus Agent for Instant Pairing"
echo "=============================================="
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

# ============================================
# STEP 1: Install system dependencies
# ============================================
echo_step "1/8: Installing system dependencies..."

case "$PKG_MANAGER" in
    apt)
        apt-get update -qq
        # Core Bluetooth packages
        apt-get install -y bluetooth bluez bluez-tools rfkill coreutils || true
        # Python packages
        apt-get install -y python3 python3-serial python3-pip || true
        # D-Bus and GLib for the agent (CRITICAL for instant pairing)
        apt-get install -y python3-dbus python3-gi python3-gi-cairo gir1.2-glib-2.0 || true
        # Development headers for PyBluez compilation
        apt-get install -y libbluetooth-dev python3-dev build-essential || true
        ;;
    dnf|yum)
        $PKG_MANAGER install -y bluez bluez-tools rfkill coreutils || true
        $PKG_MANAGER install -y python3 python3-pyserial python3-pip || true
        $PKG_MANAGER install -y python3-dbus python3-gobject || true
        $PKG_MANAGER install -y bluez-libs-devel python3-devel gcc || true
        ;;
    pacman)
        pacman -Sy --noconfirm bluez bluez-utils rfkill coreutils || true
        pacman -Sy --noconfirm python python-pyserial python-pip || true
        pacman -Sy --noconfirm python-dbus python-gobject || true
        ;;
    zypper)
        zypper install -y bluez bluez-tools rfkill coreutils || true
        zypper install -y python3 python3-pyserial python3-pip || true
        zypper install -y python3-dbus-python python3-gobject || true
        zypper install -y bluez-devel python3-devel gcc || true
        ;;
    apk)
        apk add bluez bluez-deprecated rfkill coreutils || true
        apk add python3 py3-serial py3-pip || true
        apk add py3-dbus py3-gobject3 || true
        apk add bluez-dev python3-dev build-base || true
        ;;
    *)
        echo_warn "Unknown package manager. Please install manually:"
        echo_warn "  - bluez, bluez-tools, rfkill"
        echo_warn "  - python3, python3-serial, python3-pip"
        echo_warn "  - python3-dbus, python3-gi (for D-Bus agent)"
        echo_warn "  - libbluetooth-dev, python3-dev (for PyBluez)"
        ;;
esac

echo_info "System dependencies installed"

# ============================================
# STEP 2: Install Python packages
# ============================================
echo_step "2/8: Installing Python packages..."

# PyBluez for native Bluetooth sockets (optional but preferred)
echo_info "Attempting to install PyBluez..."
pip3 install pybluez 2>/dev/null || pip3 install pybluez-updated 2>/dev/null || {
    echo_warn "PyBluez not available - will use rfcomm listen fallback"
    echo_warn "This is fine - the bridge will still work"
}

# Verify D-Bus is available
echo_info "Verifying D-Bus Python bindings..."
python3 -c "import dbus; import dbus.service; print('D-Bus OK')" 2>/dev/null && {
    echo_info "D-Bus Python bindings available - instant pairing enabled!"
} || {
    echo_warn "D-Bus bindings not available"
    echo_warn "Pairing will use bluetoothctl fallback (may require manual confirmation)"
}

# Verify GLib is available
python3 -c "from gi.repository import GLib; print('GLib OK')" 2>/dev/null && {
    echo_info "GLib Python bindings available"
} || {
    echo_warn "GLib not available - D-Bus agent may not work optimally"
}

# ============================================
# STEP 3: Create installation directory
# ============================================
echo_step "3/8: Creating installation directory..."
mkdir -p "$INSTALL_DIR"

# ============================================
# STEP 4: Install Python scripts
# ============================================
echo_step "4/8: Installing bridge scripts..."

# Main bridge script
cp "$SCRIPT_DIR/bt_arduino_bridge.py" "$INSTALL_DIR/"
chmod +x "$INSTALL_DIR/bt_arduino_bridge.py"

# Connection handler (for rfcomm watch fallback)
if [ -f "$SCRIPT_DIR/handle_connection.py" ]; then
    cp "$SCRIPT_DIR/handle_connection.py" "$INSTALL_DIR/"
    chmod +x "$INSTALL_DIR/handle_connection.py"
fi

# Reset utility
if [ -f "$SCRIPT_DIR/bt_reset.py" ]; then
    cp "$SCRIPT_DIR/bt_reset.py" "$INSTALL_DIR/"
    chmod +x "$INSTALL_DIR/bt_reset.py"
    # Also create symlink for easy access
    ln -sf "$INSTALL_DIR/bt_reset.py" /usr/local/bin/bt-reset 2>/dev/null || true
fi

echo_info "Scripts installed to $INSTALL_DIR"

# ============================================
# STEP 5: Create systemd service
# ============================================
echo_step "5/8: Creating systemd service..."

cat > "$SERVICE_FILE" << EOF
[Unit]
Description=Bluetooth to Arduino Bridge Service
Documentation=https://github.com/your-repo/bt-arduino-bridge
After=bluetooth.target dbus.service
Wants=bluetooth.target
Requires=dbus.service

[Service]
Type=simple
ExecStartPre=/bin/sleep 2
ExecStart=/usr/bin/python3 $INSTALL_DIR/bt_arduino_bridge.py
Restart=always
RestartSec=5
TimeoutStopSec=10

# Run as root for Bluetooth access
User=root
Group=root

# Environment
Environment=PYTHONUNBUFFERED=1

# Capabilities for Bluetooth
CapabilityBoundingSet=CAP_NET_ADMIN CAP_NET_RAW CAP_NET_BIND_SERVICE
AmbientCapabilities=CAP_NET_ADMIN CAP_NET_RAW CAP_NET_BIND_SERVICE

# Security hardening (optional)
NoNewPrivileges=false
ProtectSystem=false
ProtectHome=false

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=bt-arduino-bridge

[Install]
WantedBy=multi-user.target
EOF

chmod 644 "$SERVICE_FILE"
echo_info "Service file created: $SERVICE_FILE"

# ============================================
# STEP 6: Configure Bluetooth with --compat mode
# ============================================
echo_step "6/8: Configuring Bluetooth with --compat mode..."

# Create log files
touch "$LOG_FILE"
chmod 644 "$LOG_FILE"
touch /tmp/bt_bridge_debug.log /tmp/bt_bridge_commands.log
chmod 666 /tmp/bt_bridge_debug.log /tmp/bt_bridge_commands.log

# CRITICAL: Configure bluetooth.service to run with --compat flag
# This is required for sdptool to work (SPP service registration)
echo_info "Creating systemd override for bluetooth.service..."

OVERRIDE_DIR="/etc/systemd/system/bluetooth.service.d"
mkdir -p "$OVERRIDE_DIR"

cat > "$OVERRIDE_DIR/compat.conf" << 'EOF'
[Service]
# Override ExecStart to add --compat flag for sdptool support
ExecStart=
ExecStart=/usr/libexec/bluetooth/bluetoothd --compat
EOF

# If using /usr/lib path (some distros)
if [ -f "/usr/lib/bluetooth/bluetoothd" ] && [ ! -f "/usr/libexec/bluetooth/bluetoothd" ]; then
    cat > "$OVERRIDE_DIR/compat.conf" << 'EOF'
[Service]
ExecStart=
ExecStart=/usr/lib/bluetooth/bluetoothd --compat
EOF
fi

chmod 644 "$OVERRIDE_DIR/compat.conf"
echo_info "Created $OVERRIDE_DIR/compat.conf"

# Reload systemd to pick up the override
systemctl daemon-reload

# Ensure Bluetooth service is enabled and restart it with new config
if command -v systemctl &>/dev/null; then
    systemctl enable bluetooth.service 2>/dev/null || true
    systemctl restart bluetooth.service 2>/dev/null || true

    # Wait for bluetooth to start
    sleep 2

    # Verify it's running with --compat
    if pgrep -af bluetoothd | grep -q "\-\-compat"; then
        echo_info "bluetoothd running with --compat flag ✓"
    else
        echo_warn "bluetoothd may not have --compat flag"
        echo_warn "Check with: ps aux | grep bluetoothd"
    fi
fi

# Wait for Bluetooth to be ready
sleep 2

# Unblock Bluetooth
rfkill unblock bluetooth 2>/dev/null || true

# Bring up HCI
hciconfig hci0 up 2>/dev/null || true
sleep 1

# Set device name
echo_info "Setting Bluetooth device name to $BT_NAME..."
hciconfig hci0 name "$BT_NAME" 2>/dev/null || true

# Set machine-info for persistent name
echo "PRETTY_HOSTNAME=$BT_NAME" > /etc/machine-info 2>/dev/null || true

# Make discoverable and pairable
hciconfig hci0 piscan 2>/dev/null || true

# Configure via bluetoothctl
echo_info "Configuring Bluetooth settings..."
bluetoothctl << EOF 2>/dev/null || true
power on
system-alias $BT_NAME
discoverable on
discoverable-timeout 0
pairable on
agent NoInputNoOutput
default-agent
quit
EOF

echo_info "Bluetooth configured"

# ============================================
# STEP 7: Enable and start service
# ============================================
echo_step "7/8: Enabling and starting service..."

# Reload systemd
systemctl daemon-reload

# Stop any existing instance
systemctl stop bt-arduino-bridge 2>/dev/null || true
pkill -9 -f bt_arduino_bridge.py 2>/dev/null || true
sleep 1

# Enable for boot
systemctl enable bt-arduino-bridge.service

# Start service
systemctl start bt-arduino-bridge.service

# Wait and check
sleep 3

# ============================================
# STEP 8: Verify installation
# ============================================
echo_step "8/8: Verifying installation..."

SERVICE_STATUS="UNKNOWN"
if systemctl is-active --quiet bt-arduino-bridge.service; then
    SERVICE_STATUS="RUNNING"
    echo_info "Service is running!"
else
    SERVICE_STATUS="STOPPED"
    echo_warn "Service may not have started correctly"
fi

# Check what agent type is being used
AGENT_TYPE="unknown"
if python3 -c "import dbus; import dbus.service; from gi.repository import GLib" 2>/dev/null; then
    AGENT_TYPE="D-Bus (instant pairing)"
elif command -v bt-agent &>/dev/null; then
    AGENT_TYPE="bt-agent"
else
    AGENT_TYPE="bluetoothctl (fallback)"
fi

# Get Bluetooth address
BT_ADDR=$(hciconfig hci0 2>/dev/null | grep "BD Address" | awk '{print $3}' || echo "unknown")

echo
echo "=============================================="
echo " Installation Complete!"
echo "=============================================="
echo
echo "Device Configuration:"
echo "  Name:     $BT_NAME"
echo "  Address:  $BT_ADDR"
echo "  Agent:    $AGENT_TYPE"
echo "  Service:  $SERVICE_STATUS"
echo
echo "Service Commands:"
echo "  Start:    sudo systemctl start bt-arduino-bridge"
echo "  Stop:     sudo systemctl stop bt-arduino-bridge"
echo "  Status:   sudo systemctl status bt-arduino-bridge"
echo "  Restart:  sudo systemctl restart bt-arduino-bridge"
echo "  Logs:     journalctl -u bt-arduino-bridge -f"
echo
echo "Reset Utility:"
echo "  Full reset:  sudo python3 $INSTALL_DIR/bt_reset.py"
echo "  Soft reset:  sudo python3 $INSTALL_DIR/bt_reset.py --soft"
echo "  Status:      sudo python3 $INSTALL_DIR/bt_reset.py --status"
echo
echo "Log Files:"
echo "  Service:   journalctl -u bt-arduino-bridge -f"
echo "  Commands:  tail -f /tmp/bt_bridge_commands.log"
echo "  Debug:     tail -f /tmp/bt_bridge_debug.log"
echo
echo "To connect from your PC:"
echo "  1. Pair with '$BT_NAME'"
echo "  2. Connect to the Bluetooth serial port (COM port)"
echo "  3. Send commands (FWD, BWD, STOP, etc.)"
echo
