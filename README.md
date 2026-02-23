# Mecanum 4WD Robot Controller

> **Warning**
> This project is under active development and has **not been deployed to production hardware yet**. Motor signs, encoder directions, PID tuning, and kinematics formulas may require adjustment for your specific hardware setup. Use at your own risk and always test with wheels off the ground first.

Arduino Uno firmware for a 4-wheeled mecanum robot that can move in any direction — forward, backward, sideways, diagonally, or spin in place. Controlled over USB serial or with a PS2 wireless gamepad.

```
  PC (USB Serial)         Arduino Uno              PCA9685              4× TB67H450
  ─────────────           ───────────              ───────              ───────────
  FWD,100,1719   ──────►  State machine  ──I2C──►  16-ch PWM  ──8ch──►  H-Bridges
  VEL,100,0,0             PID @ 50Hz     400kHz    12-bit              (2 ch/motor)
  STOP                    Kinematics               1600Hz                   │
                               ▲                                      4× DC Motors
                          Pin-change IRQ                               9600RPM/90:1
                               │                                           │
                          4× Encoders ◄──────────────────────────── 4× Mecanum Wheels
                          4320 CPR                                     80mm dia.
```

> **Interactive docs:** Open [`docs/index.html`](docs/index.html) in a browser for visual diagrams and beginner-friendly explanations.

## Features

- **Omnidirectional movement** — 10 directions using mecanum wheel kinematics
- **Position mode** — move an exact distance, then auto-stop (e.g. "go forward 10cm")
- **Velocity mode** — continuous speed control with 20Hz odometry feedback for ROS2 integration
- **Per-motor PID** — feed-forward + PI controller at 50Hz with encoder feedback
- **Auto-calibration** — measures each motor's max speed and stores in EEPROM
- **Safety systems** — stall detection, move timeout, VEL watchdog, I2C health monitoring
- **Hardware test firmware** — separate build for verifying wiring without application logic
- **PS2 gamepad support** — optional wireless controller (disabled by default to save Flash)

## Hardware

| Component | Details |
|-----------|---------|
| MCU | Arduino Uno (ATmega328P, 16MHz, 2KB RAM, 32KB Flash) |
| PWM Driver | PCA9685 at I2C 0x60, 1600Hz, 400kHz Fast Mode |
| H-Bridges | 4× TB67H450FNG (2 channels each, on motor shield) |
| Motors | 4× DC gear motors with 4320 CPR encoders |
| Wheels | 80mm mecanum wheels |
| Geometry | 190mm wheelbase, 210mm track width |

See [Hardware Reference](docs/hardware.md) for wiring, PCA9685 registers, and H-bridge control.

## Prerequisites

### System Dependencies (Linux)

```bash
sudo apt update
sudo apt install -y build-essential python3-dev python3-tk python3-venv
```

### Serial Port Access

```bash
# Add your user to the dialout group (required to access /dev/ttyACM0)
sudo usermod -a -G dialout $USER
# Log out and log back in for this to take effect
```

### PlatformIO (Firmware Build Tool)

```bash
pip install platformio
```

## Quick Start

### 1. Build & Upload Firmware

```bash
# Build main firmware
pio run

# Upload to Arduino (auto-detects port)
pio run -t upload

# Open serial monitor (115200 baud)
pio device monitor
```

### 2. Setup Python Tools

```bash
cd tools/
./install.sh          # Creates venv, installs python3-tk + pyserial
python3 run.py        # Launch GUI controller
```

### 3. First Commands

Connect at **115200 baud**. The Arduino sends `READY` on boot.

```
FWD,100,1719      # Go forward ~10cm at speed 100
BWD,80,860        # Go backward ~5cm
LEFT,80,1719      # Strafe left ~10cm
TURN,60,500       # Rotate counter-clockwise
STOP              # Emergency stop (always safe)
VEL,100,0,0       # Continuous forward at speed 100
VEL,0,80,0        # Continuous strafe left
STOP              # Stop velocity mode
CALIB             # Calibrate motors (~10 sec)
READ              # Read encoder values
```

**Distance conversion:** `ticks = distance_mm × 17.19`

See [Protocol Reference](docs/protocol.md) for all commands, responses, and velocity mode details.

## Python Tools

| Tool | Command | Description |
|------|---------|-------------|
| **GUI Controller** | `python3 tools/run.py` | Directional pad, velocity sliders, diagnostics, live telemetry, serial monitor. Keys: WASD + QE |
| **Hardware Test CLI** | `python3 tools/hw_test.py` | Test individual motors, encoders, I2C scan — no PID or state machine |

The hardware test CLI requires the test firmware:

```bash
pio run -e hwtest -t upload    # Upload test firmware (replaces main!)
python3 tools/hw_test.py       # Run interactive CLI
pio run -t upload              # Restore main firmware when done
```

See [Tools Reference](docs/tools.md) for GUI shortcuts, CLI commands, and PS2 controller setup.

## Project Structure

```
├── src/
│   ├── app/                    # Application layer
│   │   ├── main.cpp            #   Entry point (setup + loop)
│   │   ├── robot.cpp/h         #   State machine, command dispatch, telemetry
│   │   ├── motion.cpp/h        #   PID controller, position tracking, calibration
│   │   ├── mecanum.cpp/h       #   Kinematics (direction → motor speeds)
│   │   └── serial_cmd.cpp/h    #   Serial parser (32-byte buffer, no String class)
│   ├── hal/                    # Hardware abstraction layer
│   │   ├── motor.cpp/h         #   PWM buffer, speed→duty, brake/coast
│   │   ├── encoder.cpp/h       #   Atomic reads, snapshots, distance helpers
│   │   ├── pca9685.cpp/h       #   I2C register writes, batch channels
│   │   └── timer.cpp/h         #   Timer1 CTC ISR, 50Hz tick flag
│   ├── core/                   # Shared definitions
│   │   ├── config.h            #   All constants (pins, PID, timeouts, geometry)
│   │   └── types.h             #   Enums (Direction, State, CmdType), helpers
│   └── test/                   # Hardware test firmware (env:hwtest)
│       └── main.cpp            #   Minimal firmware for wiring verification
├── lib/                        # Local libraries
│   ├── Adafruit_PWMServo/      #   PCA9685 I2C driver
│   ├── QGPMaker/               #   Encoder library (pin-change interrupts)
│   ├── PinChangeInterrupt/     #   Pin change interrupt support
│   └── PS2X/                   #   PS2 controller library
├── tools/                      # Python tools
│   ├── robot_test.py           #   GUI controller (tkinter)
│   ├── hw_test.py              #   Hardware test CLI
│   ├── install.sh              #   Dependency installer
│   ├── run.py                  #   GUI launcher
│   └── requirements.txt        #   Python dependencies (pyserial)
├── docs/                       # Documentation
│   ├── index.html              #   Interactive visual docs (open in browser)
│   ├── architecture.md         #   Software layers, state machine, timing
│   ├── protocol.md             #   Serial commands, velocity mode, responses
│   ├── hardware.md             #   Wiring, PCA9685, H-bridges, encoders
│   ├── motion-control.md       #   Kinematics, PID, calibration, speed ramping
│   ├── tools.md                #   GUI, CLI, test firmware, PS2 controller
│   ├── configuration.md        #   All config.h constants, safety, troubleshooting
│   ├── Motor-Shield-V5.3SCH.pdf  # Motor shield schematic
│   └── images/                 #   Reference photos and diagrams
└── platformio.ini              # Build configuration (env:uno + env:hwtest)
```

## Documentation

| Document | What's Inside |
|----------|--------------|
| [Architecture](docs/architecture.md) | Three-layer design, command flow trace, main loop timing, state machine, boot sequence |
| [Protocol](docs/protocol.md) | All serial commands, response messages, velocity mode, parser internals, session example |
| [Hardware](docs/hardware.md) | Component details, PCA9685 registers, H-bridge control, motor wiring map, encoder specs |
| [Motion Control](docs/motion-control.md) | Mecanum kinematics, per-motor PID (FF+PI), speed ramping, calibration sequence |
| [Tools](docs/tools.md) | GUI features & shortcuts, hardware test CLI, test firmware commands, PS2 controller |
| [Configuration](docs/configuration.md) | All `config.h` constants, safety systems, troubleshooting guide |
| [Interactive Docs](docs/index.html) | Visual diagrams and beginner explanations (open in browser) |
| [Motor Shield Schematic](docs/Motor-Shield-V5.3SCH.pdf) | V5.3 motor shield circuit diagram |

## Build Stats

| Resource | Usage |
|----------|-------|
| Flash | 53.0% (17,096 / 32,256 bytes) |
| RAM | 53.0% (1,085 / 2,048 bytes) |

## License

MIT
