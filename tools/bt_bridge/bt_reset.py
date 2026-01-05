#!/usr/bin/env python3
"""
Bluetooth Reset Utility
Cleans up Bluetooth connections and pairing data for fresh connections.

Use this script when switching between different PCs to ensure clean connections.

Usage:
    sudo python3 bt_reset.py           # Full reset (unpair all + restart)
    sudo python3 bt_reset.py --soft    # Soft reset (just disconnect and restart service)
    sudo python3 bt_reset.py --unpair  # Only unpair all devices
    sudo python3 bt_reset.py --status  # Show current BT status
    sudo python3 bt_reset.py --diag    # Full diagnostics
"""

import subprocess
import sys
import os
import argparse
import time
from pathlib import Path
from datetime import datetime

# Colors for terminal output
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    CYAN = '\033[0;36m'
    BOLD = '\033[1m'
    NC = '\033[0m'  # No Color

def print_info(msg):
    print(f"{Colors.GREEN}[INFO]{Colors.NC} {msg}")

def print_warn(msg):
    print(f"{Colors.YELLOW}[WARN]{Colors.NC} {msg}")

def print_error(msg):
    print(f"{Colors.RED}[ERROR]{Colors.NC} {msg}")

def print_step(msg):
    print(f"{Colors.CYAN}[STEP]{Colors.NC} {msg}")


def run_cmd(cmd, shell=False, capture=True, check=False, timeout=10):
    """Run a command and return (success, stdout, stderr)"""
    try:
        if isinstance(cmd, str) and not shell:
            cmd = cmd.split()
        result = subprocess.run(
            cmd,
            shell=shell,
            capture_output=capture,
            text=True,
            timeout=timeout,
            check=check
        )
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        print_warn(f"Command timed out: {cmd}")
        return False, "", "Timeout"
    except Exception as e:
        return False, "", str(e)


def run_bluetoothctl(commands, timeout=5):
    """Run bluetoothctl commands safely with timeout"""
    try:
        proc = subprocess.Popen(
            ['bluetoothctl'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        stdout, stderr = proc.communicate(input=commands + "\nquit\n", timeout=timeout)
        return True, stdout, stderr
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        print_warn("bluetoothctl timed out, killed")
        return False, "", "Timeout"
    except Exception as e:
        return False, "", str(e)


def get_paired_devices():
    """Get list of paired device addresses with names"""
    devices = []
    success, stdout, _ = run_cmd("bluetoothctl paired-devices", timeout=5)
    if success and stdout:
        for line in stdout.strip().split('\n'):
            if line.startswith("Device "):
                parts = line.split()
                if len(parts) >= 3:
                    addr = parts[1]
                    name = ' '.join(parts[2:])
                    devices.append((addr, name))
                elif len(parts) >= 2:
                    devices.append((parts[1], "Unknown"))
    return devices


def get_connected_devices():
    """Get list of currently connected device addresses"""
    devices = []
    paired = get_paired_devices()
    for addr, name in paired:
        success, stdout, _ = run_cmd(f"bluetoothctl info {addr}", timeout=3)
        if success and "Connected: yes" in stdout:
            devices.append((addr, name))
    return devices


def disconnect_device(addr):
    """Disconnect a specific device"""
    print(f"  Disconnecting {addr}...")
    run_cmd(f"bluetoothctl disconnect {addr}", timeout=5)
    time.sleep(0.3)


def remove_device(addr):
    """Remove/unpair a specific device"""
    print(f"  Removing {addr}...")
    # First disconnect
    run_cmd(f"bluetoothctl disconnect {addr}", timeout=3)
    time.sleep(0.2)
    # Then remove
    run_cmd(f"bluetoothctl remove {addr}", timeout=5)
    time.sleep(0.2)


def release_rfcomm():
    """Release all rfcomm devices"""
    print_step("Releasing rfcomm devices...")
    for i in range(5):
        run_cmd(f"rfcomm release /dev/rfcomm{i}")
    # Also try killing any rfcomm processes
    run_cmd("pkill -9 rfcomm")
    time.sleep(0.5)


def stop_bt_service():
    """Stop the BT bridge service and all related processes"""
    print_step("Stopping BT bridge service...")

    # First try graceful stop with timeout
    try:
        subprocess.run(
            ["systemctl", "stop", "bt-arduino-bridge"],
            timeout=5,
            capture_output=True
        )
    except subprocess.TimeoutExpired:
        print_warn("Service stop timed out, forcing...")
    except Exception:
        pass

    # Force kill everything related
    processes_to_kill = [
        "bt_arduino_bridge.py",
        "handle_connection.py",
        "rfcomm",
        "bluetoothctl",
        "bt-agent"
    ]

    for proc in processes_to_kill:
        run_cmd(f"pkill -9 -f {proc}")

    time.sleep(1)
    print_info("Service stopped")


def restart_bluetooth():
    """Restart the Bluetooth subsystem"""
    print_step("Restarting Bluetooth subsystem...")

    # Stop services
    run_cmd("systemctl stop bluetooth")
    time.sleep(1)

    # Reset HCI
    run_cmd("hciconfig hci0 down")
    time.sleep(0.5)
    run_cmd("hciconfig hci0 reset")
    time.sleep(0.5)

    # Unblock rfkill
    run_cmd("rfkill unblock bluetooth")
    time.sleep(0.5)

    # Start services
    run_cmd("systemctl start bluetooth")
    time.sleep(2)

    # Bring up HCI
    run_cmd("hciconfig hci0 up")
    time.sleep(1)

    # Make discoverable and pairable
    run_cmd("hciconfig hci0 piscan")

    # Set device name
    run_cmd("hciconfig hci0 name 'Arduino_BT_Bridge'")

    print_info("Bluetooth restarted")


def start_bt_service():
    """Start the BT bridge service"""
    print_step("Starting BT bridge service...")

    # Check if service exists
    success, stdout, _ = run_cmd("systemctl list-unit-files bt-arduino-bridge.service")
    if not success or "bt-arduino-bridge" not in stdout:
        print_warn("Service not installed. Run install.sh first.")
        return False

    # Check if service is masked
    success, stdout, _ = run_cmd("systemctl is-enabled bt-arduino-bridge")
    if "masked" in stdout.lower():
        print_info("Service is masked, unmasking...")
        run_cmd("systemctl unmask bt-arduino-bridge")
        time.sleep(0.5)

    # Enable and start the service
    run_cmd("systemctl enable bt-arduino-bridge")
    run_cmd("systemctl start bt-arduino-bridge")
    time.sleep(3)

    # Verify it started
    success, stdout, _ = run_cmd("systemctl is-active bt-arduino-bridge")
    if success and "active" in stdout:
        print_info("Service started successfully")
        return True
    else:
        print_warn("Service may not have started. Check: systemctl status bt-arduino-bridge")
        return False


def check_dbus_agent():
    """Check if D-Bus agent is available"""
    try:
        import dbus
        import dbus.service
        from gi.repository import GLib
        return True, "D-Bus agent available (instant pairing)"
    except ImportError as e:
        return False, f"D-Bus not available: {e}"


def show_status():
    """Show current Bluetooth status"""
    print(f"\n{Colors.BOLD}=== Bluetooth Status ==={Colors.NC}\n")

    # HCI status
    print(f"{Colors.CYAN}HCI Adapter:{Colors.NC}")
    success, stdout, _ = run_cmd("hciconfig hci0")
    if success:
        # Parse key info
        for line in stdout.split('\n'):
            line = line.strip()
            if "BD Address" in line or "UP RUNNING" in line or "DOWN" in line:
                print(f"  {line}")

        # Check if UP
        if "UP RUNNING" in stdout:
            print(f"  Status: {Colors.GREEN}UP{Colors.NC}")
        else:
            print(f"  Status: {Colors.RED}DOWN{Colors.NC}")
    else:
        print(f"  {Colors.RED}hci0 not available{Colors.NC}")

    # Paired devices
    print(f"\n{Colors.CYAN}Paired Devices:{Colors.NC}")
    paired = get_paired_devices()
    if paired:
        for addr, name in paired:
            success, stdout, _ = run_cmd(f"bluetoothctl info {addr}", timeout=3)
            connected = "Connected: yes" in stdout if success else False
            status = f"{Colors.GREEN}[CONNECTED]{Colors.NC}" if connected else ""
            print(f"  {addr} - {name} {status}")
    else:
        print("  No paired devices")

    # RFCOMM status
    print(f"\n{Colors.CYAN}RFCOMM Connections:{Colors.NC}")
    success, stdout, _ = run_cmd("rfcomm -a")
    if success and stdout.strip():
        for line in stdout.strip().split('\n'):
            print(f"  {line}")
    else:
        print("  No active rfcomm connections")

    # Service status
    print(f"\n{Colors.CYAN}BT Bridge Service:{Colors.NC}")

    # Check if installed
    success, stdout, _ = run_cmd("systemctl list-unit-files bt-arduino-bridge.service")
    if "bt-arduino-bridge" not in stdout:
        print(f"  Status: {Colors.YELLOW}NOT INSTALLED{Colors.NC}")
        print("  Run: sudo ./install.sh")
    else:
        # Check if masked
        _, enabled_stdout, _ = run_cmd("systemctl is-enabled bt-arduino-bridge")
        if "masked" in enabled_stdout.lower():
            print(f"  Status: {Colors.YELLOW}MASKED{Colors.NC}")
            print("  Run: sudo systemctl unmask bt-arduino-bridge")
        else:
            success, stdout, _ = run_cmd("systemctl is-active bt-arduino-bridge")
            if success and "active" in stdout:
                print(f"  Status: {Colors.GREEN}RUNNING{Colors.NC}")
            else:
                print(f"  Status: {Colors.RED}STOPPED{Colors.NC}")

    # D-Bus agent status
    print(f"\n{Colors.CYAN}Pairing Agent:{Colors.NC}")
    available, msg = check_dbus_agent()
    if available:
        print(f"  {Colors.GREEN}{msg}{Colors.NC}")
    else:
        print(f"  {Colors.YELLOW}{msg}{Colors.NC}")
        print("  Install: sudo apt install python3-dbus python3-gi")

    print()


def show_diagnostics():
    """Show full diagnostics"""
    print(f"\n{Colors.BOLD}=== Full Bluetooth Diagnostics ==={Colors.NC}\n")
    print(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

    # System info
    print(f"{Colors.CYAN}System:{Colors.NC}")
    success, stdout, _ = run_cmd("uname -a")
    if success:
        print(f"  {stdout.strip()}")

    # Bluetooth packages
    print(f"\n{Colors.CYAN}Bluetooth Packages:{Colors.NC}")
    for pkg in ["bluez", "bluez-tools", "rfkill"]:
        success, _, _ = run_cmd(f"which {pkg.replace('-tools', 'ctl')}")
        status = f"{Colors.GREEN}OK{Colors.NC}" if success else f"{Colors.RED}MISSING{Colors.NC}"
        print(f"  {pkg}: {status}")

    # Python packages
    print(f"\n{Colors.CYAN}Python Packages:{Colors.NC}")
    packages = [
        ("serial", "pyserial"),
        ("dbus", "python3-dbus"),
        ("gi.repository.GLib", "python3-gi"),
        ("bluetooth", "pybluez (optional)")
    ]
    for module, name in packages:
        try:
            __import__(module.split('.')[0])
            print(f"  {name}: {Colors.GREEN}OK{Colors.NC}")
        except ImportError:
            print(f"  {name}: {Colors.YELLOW}MISSING{Colors.NC}")

    # HCI info
    print(f"\n{Colors.CYAN}HCI Details:{Colors.NC}")
    success, stdout, _ = run_cmd("hciconfig -a hci0")
    if success:
        print(stdout)
    else:
        print(f"  {Colors.RED}hci0 not available{Colors.NC}")

    # rfkill status
    print(f"{Colors.CYAN}RFKill Status:{Colors.NC}")
    success, stdout, _ = run_cmd("rfkill list bluetooth")
    if success:
        print(stdout)

    # bluetoothd status
    print(f"{Colors.CYAN}Bluetoothd:{Colors.NC}")
    success, stdout, _ = run_cmd("pgrep -a bluetoothd")
    if success and stdout.strip():
        print(f"  {stdout.strip()}")
        if "--compat" in stdout:
            print(f"  {Colors.GREEN}Running with --compat (SDP enabled){Colors.NC}")
        else:
            print(f"  {Colors.YELLOW}Running without --compat{Colors.NC}")
    else:
        print(f"  {Colors.RED}NOT RUNNING{Colors.NC}")

    # SDP check
    print(f"\n{Colors.CYAN}SDP Service:{Colors.NC}")
    success, stdout, stderr = run_cmd("sdptool browse local", timeout=5)
    if success:
        if "Serial Port" in stdout:
            print(f"  {Colors.GREEN}SPP service registered{Colors.NC}")
        else:
            print(f"  {Colors.YELLOW}SPP service NOT registered{Colors.NC}")
    else:
        print(f"  {Colors.RED}SDP not accessible{Colors.NC}")
        if stderr == "Timeout":
            print("  Hint: sdptool timed out, bluetoothd may be unresponsive")
        elif "Failed to connect" in stderr:
            print("  Hint: bluetoothd needs --compat flag")

    # Service logs (last 10 lines)
    print(f"\n{Colors.CYAN}Recent Service Logs:{Colors.NC}")
    success, stdout, _ = run_cmd("journalctl -u bt-arduino-bridge -n 10 --no-pager")
    if success and stdout.strip():
        for line in stdout.strip().split('\n'):
            print(f"  {line}")
    else:
        print("  No logs available")

    # Show status at the end
    show_status()


def soft_reset():
    """Soft reset - disconnect all and restart service without unpairing"""
    print(f"\n{Colors.BOLD}=== Soft Bluetooth Reset ==={Colors.NC}\n")

    # Stop service
    stop_bt_service()

    # Release rfcomm
    release_rfcomm()

    # Disconnect all connected devices
    connected = get_connected_devices()
    if connected:
        print_step("Disconnecting devices...")
        for addr, name in connected:
            disconnect_device(addr)

    # Restart Bluetooth
    restart_bluetooth()

    # Start service
    start_bt_service()

    print(f"\n{Colors.GREEN}Soft reset complete!{Colors.NC}")
    print("Existing pairings preserved. You can now connect from a different PC.")
    print("(The new PC will need to pair first)")


def unpair_all():
    """Unpair/remove all paired devices"""
    print(f"\n{Colors.BOLD}=== Unpair All Devices ==={Colors.NC}\n")

    # Stop service first
    stop_bt_service()
    release_rfcomm()

    # Get and remove all paired devices
    paired = get_paired_devices()
    if paired:
        print_info(f"Found {len(paired)} paired device(s)")
        for addr, name in paired:
            remove_device(addr)
        print_info("All devices unpaired")
    else:
        print_info("No paired devices found")


def full_reset():
    """Full reset - unpair all devices and restart everything"""
    print(f"\n{Colors.BOLD}=== Full Bluetooth Reset ==={Colors.NC}\n")

    # Stop service
    stop_bt_service()

    # Release rfcomm
    release_rfcomm()

    # Unpair all devices
    paired = get_paired_devices()
    if paired:
        print_step(f"Removing {len(paired)} paired device(s)...")
        for addr, name in paired:
            remove_device(addr)

    # Restart Bluetooth
    restart_bluetooth()

    # Configure for new connections
    print_step("Configuring for new connections...")
    bt_commands = """power on
discoverable on
discoverable-timeout 0
pairable on
agent NoInputNoOutput
default-agent"""
    success, _, _ = run_bluetoothctl(bt_commands, timeout=8)
    if not success:
        print_warn("bluetoothctl configuration may have failed")

    # Re-register SPP service
    print_step("Re-registering SPP service...")
    # First check if SDP is accessible
    success, _, stderr = run_cmd("sdptool browse local")
    if not success or "Failed to connect" in stderr:
        print_warn("SDP not accessible, service will handle this")
    else:
        run_cmd("sdptool add --channel=1 SP")
        print_info("SPP service registered")

    # Start service
    start_bt_service()

    print(f"\n{'=' * 50}")
    print(f"{Colors.GREEN}Full reset complete!{Colors.NC}")
    print("=" * 50)
    print("\nThe device is now ready for fresh connections.")
    print("On your PC:")
    print("  1. Remove 'Arduino_BT_Bridge' from paired devices (if exists)")
    print("  2. Scan for new devices")
    print("  3. Pair with 'Arduino_BT_Bridge'")
    print("  4. Connect using the Bluetooth serial port")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Bluetooth Reset Utility for clean connections when switching PCs",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  sudo python3 bt_reset.py           # Full reset (recommended when switching PCs)
  sudo python3 bt_reset.py --soft    # Soft reset (keep pairings, just restart)
  sudo python3 bt_reset.py --unpair  # Only unpair all devices
  sudo python3 bt_reset.py --status  # Show current status
  sudo python3 bt_reset.py --diag    # Full diagnostics
"""
    )

    parser.add_argument('--soft', action='store_true',
                        help='Soft reset: disconnect and restart without unpairing')
    parser.add_argument('--unpair', action='store_true',
                        help='Only unpair all devices')
    parser.add_argument('--status', action='store_true',
                        help='Show current Bluetooth status')
    parser.add_argument('--diag', '--diagnostics', action='store_true',
                        help='Show full diagnostics')

    args = parser.parse_args()

    # Check for root (except for status/diag which can run as user)
    if os.geteuid() != 0 and not (args.status or args.diag):
        print_error("This script requires root privileges")
        print("Please run with: sudo python3 bt_reset.py")
        return 1

    if args.diag:
        show_diagnostics()
    elif args.status:
        show_status()
    elif args.soft:
        soft_reset()
        show_status()
    elif args.unpair:
        unpair_all()
        show_status()
    else:
        full_reset()
        show_status()

    return 0


if __name__ == "__main__":
    sys.exit(main())
