# Mecanum 4WD Robot - Control Tools

Python GUI and test tools for controlling the Mecanum wheel robot over USB serial.

---

## Installation

The installer works on **Windows, Linux, and macOS** — one script for all platforms.

### Requirements
- Python 3.8 or newer — download from [python.org](https://www.python.org/downloads/)

### Run the installer

**Windows:**
```powershell
py install.py
```

**Linux / macOS:**
```bash
python3 install.py
```

**Options:**
```bash
py install.py --clean   # wipe venv and reinstall from scratch
py install.py --check   # verify existing install only
```

The installer will:
1. Check your Python version and tkinter
2. Create a virtual environment (`venv/`)
3. Upgrade pip and install all dependencies
4. Confirm everything is working

---

## Running the Robot GUI

After installation, launch the GUI with:

**Windows:**
```powershell
venv\Scripts\python.exe robot_test.py
```

**Linux / macOS:**
```bash
venv/bin/python robot_test.py
```

---

## Files

| File | Description |
|------|-------------|
| `robot_test.py` | Main GUI application |
| `install.py` | Cross-platform setup script |
| `requirements.txt` | Python dependencies |
| `hw_test.py` | Hardware test CLI (motors & encoders) |
| `run.py` | Alternative launcher |
| `START_HERE.txt` | Quick reference guide |

---

## GUI Features

- **Connection**: Auto-detects Arduino/USB serial ports
- **Position Control**: Direction buttons with speed and tick parameters
- **Velocity Control**: Real-time streaming at 10 Hz with sliders
- **Diagnostics**: Per-motor PWM test and encoder test
- **Telemetry**: Live encoder and odometry display
- **Serial Monitor**: Full-color console with send/receive history

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| W / S | Forward / Backward |
| A / D | Strafe Left / Right |
| Q / E | Turn CCW / CW |
| 7 / 9 / 1 / 3 | Diagonal moves |
| Space / Esc | Emergency Stop |
| R | Read Encoders |

---

## Serial Commands

```
FWD,<speed>,<ticks>       Move forward
BWD,<speed>,<ticks>       Move backward
LEFT,<speed>,<ticks>      Strafe left
RIGHT,<speed>,<ticks>     Strafe right
TURN,<speed>,<ticks>      Turn (positive=left, negative=right)
DIAGFL,<speed>,<ticks>    Diagonal front-left
VEL,<vx>,<vy>,<wz>        Continuous velocity control
TMOTOR,<motor>,<pwm>      Test single motor (FL/FR/RL/RR)
TENC                      Test encoders
CALIB                     Calibrate motors
READ                      Read encoder values
STOP                      Emergency stop
```

Parameters:
- `speed`: 20–255
- `ticks`: encoder ticks (e.g. 1000)

---

## Troubleshooting

### `py` not found (Windows)
Use `python` instead, or install Python from [python.org](https://www.python.org/downloads/) and tick **"Add Python to PATH"**.

### Cygwin/WSL conflict (Windows)
If `python` resolves to a Cygwin binary, use the `py` launcher:
```powershell
py install.py
```

### Serial port permission denied (Linux)
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### GUI won't start — tkinter missing (Linux)
```bash
sudo apt install python3-tk        # Debian / Ubuntu
sudo dnf install python3-tkinter   # Fedora / RHEL
```

### Reinstall from scratch
```bash
py install.py --clean
```
