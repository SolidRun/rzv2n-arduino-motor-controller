# Mecanum 4WD Robot - Control Tools

Python GUI and test tools for controlling the Mecanum wheel robot.

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
| `hw_test.py` | Hardware test CLI (motors & encoders) |

## GUI Features

- **Serial Connection**: Auto-detects Arduino ports
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
```

### Python
- pyserial >= 3.5

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

### Virtual Environment Issues
```bash
rm -rf venv
./install.sh
```
