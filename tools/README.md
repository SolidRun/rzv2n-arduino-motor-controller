# Mecanum 4WD Robot - Control Tools

Python GUI and Bluetooth bridge for controlling the Mecanum wheel robot.

## Quick Start

```bash
# Setup (first time)
./install.sh

# Run GUI
python3 run.py
```

## Files

| File | Description |
|------|-------------|
| `robot_test.py` | Main GUI application |
| `run.py` | GUI launcher (uses venv) |
| `install.sh` | One-command setup script |
| `requirements.txt` | Python dependencies |
| `START_HERE.txt` | Quick reference guide |
| `bt_bridge/` | Bluetooth-to-Serial bridge service |

## GUI Features

- **Serial Connection**: Auto-detects Arduino ports
- **Bluetooth Connection**: Scan and connect to BT devices
- **Keyboard Controls**: WASD + QE for movement
- **Live Monitor**: Real-time serial output

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| W/S | Forward/Backward |
| A/D | Strafe Left/Right |
| Q/E | Turn Left/Right |
| Space | Emergency Stop |
| R | Read Encoders |

## Bluetooth Bridge

For wireless control from any Bluetooth device:

```bash
# Install as system service
sudo bt_bridge/install.sh

# Check status
sudo systemctl status bt-arduino-bridge

# View logs
tail -f /tmp/bt_bridge_commands.log
```

The bridge creates a Bluetooth device named "Arduino_BT_Bridge" that forwards commands to Arduino via USB.

### Switching Between PCs

If you have connection issues when connecting from a different PC:

```bash
# Run reset on the bridge device (iMX8P/Raspberry Pi)
sudo python3 bt_bridge/bt_reset.py

# Options:
#   --status   Show current BT status
#   --soft     Soft reset (keep pairings)
#   --unpair   Only unpair all devices
#   (no args)  Full reset (recommended)
```

Then on the new PC: remove old pairing, scan, and pair again.

See `bt_bridge/USAGE.txt` for details.

## Serial Commands

```
FWD,<speed>,<ticks>    Move forward
BWD,<speed>,<ticks>    Move backward
TURN,<speed>,<ticks>   Turn (positive=left, negative=right)
STOP                   Emergency stop
READ                   Read encoder values
SYNC                   Calibrate motors
FWDS                   Smart forward with PID
```

Parameters:
- `speed`: 20-255
- `ticks`: encoder ticks (e.g., 1000)

## Requirements

### System (Linux)
```bash
sudo apt install python3 python3-pip python3-tk python3-venv
sudo apt install libbluetooth-dev  # For Bluetooth support
```

### Python
- pyserial >= 3.5
- pybluez-updated (optional, for Bluetooth)

## Troubleshooting

### Serial Port Permission Denied
```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

### GUI Won't Start
```bash
sudo apt install python3-tk
```

### Bluetooth Not Available
Serial/USB works without Bluetooth - use that mode if BT fails.

To enable Bluetooth:
```bash
sudo apt install libbluetooth-dev build-essential python3-dev
./install.sh  # Re-run setup
```

### Virtual Environment Issues
```bash
rm -rf venv
./install.sh
```
