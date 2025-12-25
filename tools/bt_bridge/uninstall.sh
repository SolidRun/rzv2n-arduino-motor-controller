#!/bin/bash
#
# Uninstall Script for Bluetooth-Arduino Bridge
# Run as root: sudo ./uninstall.sh
#

set -e

INSTALL_DIR="/opt/bt_arduino_bridge"
SERVICE_FILE="/etc/systemd/system/bt-arduino-bridge.service"
PID_FILE="/var/run/bt_arduino_bridge.pid"

# Log files (matching bt_arduino_bridge.sh)
DEBUG_LOG="/tmp/bt_bridge_debug.log"
CMD_LOG="/tmp/bt_bridge_commands.log"
SERVICE_LOG="/tmp/bt_arduino_bridge.log"

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

echo_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
echo_error() { echo -e "${RED}[ERROR]${NC} $1"; }

if [ "$EUID" -ne 0 ]; then
    echo_error "Please run as root (sudo ./uninstall.sh)"
    exit 1
fi

echo "Uninstalling Bluetooth-Arduino Bridge..."

# Stop service
echo_info "Stopping service..."
systemctl stop bt-arduino-bridge.service 2>/dev/null || true

# Disable service
echo_info "Disabling service..."
systemctl disable bt-arduino-bridge.service 2>/dev/null || true

# Remove service file
echo_info "Removing service file..."
rm -f "$SERVICE_FILE"

# Reload systemd
systemctl daemon-reload

# Remove installation directory
echo_info "Removing installation files..."
rm -rf "$INSTALL_DIR"

# Remove PID file
rm -f "$PID_FILE"

# Ask about log files
read -r -p "Remove log files (/tmp/bt_bridge_*.log)? [y/N]: " remove_log
if [[ "$remove_log" =~ ^[Yy]$ ]]; then
    rm -f "$DEBUG_LOG" "$CMD_LOG" "$SERVICE_LOG"
    echo_info "Log files removed"
else
    echo_info "Log files kept in /tmp/"
fi

echo
echo_info "Uninstallation complete!"
