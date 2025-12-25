# Mecanum 4WD Robot Controller

Arduino-based Mecanum wheel robot controller with modular architecture, real-time motor control, and wireless Bluetooth connectivity.

## Visual Documentation

Open the interactive HTML documentation for visual diagrams and easy-to-understand explanations:

```bash
# Linux
xdg-open docs/index.html

# macOS
open docs/index.html

# Windows
start docs/index.html
```

Or simply open `docs/index.html` in your web browser.

---

## Table of Contents

- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Project Structure](#project-structure)
- [Architecture Overview](#architecture-overview)
- [Building and Uploading](#building-and-uploading)
- [Serial Command Protocol](#serial-command-protocol)
- [GUI Control Application](#gui-control-application)
- [Bluetooth Bridge (Wireless Control)](#bluetooth-bridge-wireless-control)
- [PS2 Controller Support](#ps2-controller-support)
- [Motor Control System](#motor-control-system)
- [Encoder System](#encoder-system)
- [Calibration](#calibration)
- [Testing Guide](#testing-guide)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Features

- **4 DC Motors** with quadrature encoder feedback (4320 counts/revolution)
- **Mecanum Wheel Drive** - omnidirectional movement (forward, backward, strafe, rotate, diagonals)
- **PS2 Controller** support (optional wireless gamepad)
- **Serial Command Interface** - text-based protocol over USB
- **Bluetooth Bridge** - wireless control via imx8mp or similar Linux SBC
- **Timer-Based Control** - 50Hz fixed-rate control loop
- **Motor Calibration** - automatic gain compensation for balanced movement
- **Python GUI** - visual control interface with keyboard shortcuts

---

## Hardware Requirements

### Main Components

| Component | Description |
|-----------|-------------|
| **Arduino Uno** | Main controller (ATmega328P, 16MHz) |
| **PCA9685 Motor Shield** | I2C PWM driver at address 0x60, 1600Hz PWM |
| **4x TB67H450 H-Bridge** | Motor drivers (integrated on shield) |
| **4x DC Motors** | With quadrature encoders (4320 CPR) |
| **PS2 Controller** | Wireless gamepad (optional) |

### Wiring

**PCA9685 Motor Channels:**
```
Motor 0 (Front-Left):  CH8 (IN1), CH9 (IN2)
Motor 1 (Front-Right): CH10 (IN1), CH11 (IN2)
Motor 2 (Rear-Left):   CH15 (IN1), CH14 (IN2)
Motor 3 (Rear-Right):  CH13 (IN1), CH12 (IN2)
```

**PS2 Controller Pins:**
```
CLK: Pin 13
CMD: Pin 11
ATT: Pin 10
DAT: Pin 12
```

### Power Requirements

- **Arduino**: USB power (5V)
- **Motors**: External power supply (voltage depends on motor specifications)

---

## Project Structure

```
rzv2n-arduino-motor-controller/
├── src/
│   ├── app/                    # Application layer
│   │   ├── main.cpp           # Entry point
│   │   ├── robot.cpp/.h       # State machine coordinator
│   │   ├── motion.cpp/.h      # Motion control with feedback
│   │   ├── mecanum.cpp/.h     # Kinematics calculations
│   │   └── serial_cmd.cpp/.h  # Command parsing
│   │
│   ├── hal/                    # Hardware Abstraction Layer
│   │   ├── motor.cpp/.h       # Motor driver interface
│   │   ├── encoder.cpp/.h     # Encoder input handling
│   │   ├── pca9685.cpp/.h     # I2C PWM driver
│   │   └── timer.cpp/.h       # 50Hz control loop timer
│   │
│   └── core/                   # Core definitions
│       ├── config.h           # All constants and pins
│       └── types.h            # Enums and structs
│
├── lib/                        # External libraries
│   ├── Adafruit_PWMServo/     # I2C PWM driver
│   ├── PinChangeInterrupt/    # Encoder interrupts
│   ├── PS2X/                  # PS2 controller library
│   └── QGPMaker/              # Motor shield encoder library
│
├── tools/                      # Control tools
│   ├── robot_test.py          # GUI control application
│   ├── run.py                 # GUI launcher
│   └── bt_bridge/             # Bluetooth bridge service
│       ├── bt_arduino_bridge.py   # Main service daemon
│       └── handle_connection.py   # Per-connection handler
│
└── platformio.ini             # Build configuration
```

---

## Architecture Overview

### Layered Design

```
┌─────────────────────────────────────────────────────┐
│              APPLICATION LAYER (src/app/)           │
│  robot.cpp → motion.cpp → mecanum.cpp → serial_cmd  │
└─────────────────────────────────────────────────────┘
                          │
┌─────────────────────────────────────────────────────┐
│         HARDWARE ABSTRACTION LAYER (src/hal/)       │
│     motor.cpp ← encoder.cpp ← timer.cpp ← pca9685   │
└─────────────────────────────────────────────────────┘
                          │
┌─────────────────────────────────────────────────────┐
│                    HARDWARE                         │
│   Arduino Uno ← PCA9685 ← TB67H450 ← Motors/Encoders│
└─────────────────────────────────────────────────────┘
```

### Control Loop (50Hz)

```
Timer1 ISR fires every 20ms
        ↓ sets flag
    main loop
        ↓ checks flag
Motion::update()
        ↓
┌───────────────────────────────────────┐
│ 1. Read encoder snapshot              │
│ 2. Check if target reached            │
│ 3. Calculate slowdown ramp            │
│ 4. Apply motor synchronization        │
│ 5. Update motor PWM via PCA9685       │
└───────────────────────────────────────┘
```

### Key Design Decisions

- **ISR-Safe**: Timer ISR only sets a flag; no I2C operations in interrupt context
- **Atomic Reads**: 32-bit encoder values protected with cli()/SREG
- **I2C Fast Mode**: 400kHz for reduced motor update latency
- **Non-Blocking**: Character buffers, state machines, no String class
- **Buffered Updates**: All 8 PWM channels written in single I2C transaction

---

## Building and Uploading

### Prerequisites

- [PlatformIO CLI](https://platformio.org/install/cli) or PlatformIO IDE
- Python 3.6+ (for GUI tools)

### Build and Upload

```bash
# Build only
pio run

# Build and upload to Arduino
pio run -t upload

# Monitor serial output
pio device monitor
```

### Build Information

- **Flash Usage**: ~14.5KB (45% of 32KB)
- **RAM Usage**: ~929 bytes (45% of 2KB)
- **Serial Baud**: 9600

---

## Serial Command Protocol

### Command Format

Text-based, newline-terminated: `COMMAND,param1,param2\n`

### Available Commands

| Command | Parameters | Example | Description |
|---------|------------|---------|-------------|
| `FWD` | speed, ticks | `FWD,100,1000` | Move forward |
| `BWD` | speed, ticks | `BWD,100,1000` | Move backward |
| `LEFT` | speed, ticks | `LEFT,80,500` | Strafe left |
| `RIGHT` | speed, ticks | `RIGHT,80,500` | Strafe right |
| `TURN` | speed, ticks | `TURN,80,500` | Rotate (positive=CCW, negative=CW) |
| `STOP` | - | `STOP` | Emergency stop (applies brake) |
| `READ` | - | `READ` | Read encoder values (returns 4 lines) |
| `SYNC` | - | `SYNC` | Calibrate motors (blocking, ~10s) |

### Parameters

- **speed**: 20-255 (PWM duty cycle percentage)
- **ticks**: Encoder counts to travel (4320 = 1 full wheel revolution)

### Response Messages

| Response | Meaning |
|----------|---------|
| `READY` | System initialized, ready for commands |
| `DONE` | Command completed successfully |
| `BUSY` | Cannot process (movement in progress) |
| `ERROR: message` | Command failed with reason |
| `<number>` | Encoder value (4 lines for READ command) |

### Example Session

```
> READY                    # Arduino sends on boot
< READ                     # Request encoder values
> 0                        # FL encoder
> 0                        # FR encoder
> 0                        # RL encoder
> 0                        # RR encoder
< FWD,100,2000             # Move forward
> DONE                     # Movement completed
< SYNC                     # Calibrate
> DONE                     # Calibration done
```

---

## GUI Control Application

### Installation

```bash
cd tools
pip3 install pyserial tkinter
# Optional for Bluetooth:
pip3 install pybluez
```

### Running the GUI

```bash
python3 tools/run.py
# or directly:
python3 tools/robot_test.py
```

### GUI Features

1. **Connection Panel**
   - Auto-detect Arduino ports
   - Manual port selection
   - Bluetooth device scanning and connection
   - Connect/Disconnect buttons

2. **Control Panel**
   - Speed slider (20-255)
   - Ticks slider (100-10000)
   - 8 directional movement buttons
   - STOP button (emergency)
   - READ (encoders) and SYNC (calibrate) buttons

3. **Communication Log**
   - Color-coded messages (TX=blue, RX=green, error=red)
   - Auto-scroll with toggle
   - Manual command entry field

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `W` | Forward |
| `S` | Backward |
| `A` | Strafe Left |
| `D` | Strafe Right |
| `Q` | Rotate Left (CCW) |
| `E` | Rotate Right (CW) |
| `Space` | STOP |
| `R` | Read Encoders |

---

## Bluetooth Bridge (Wireless Control)

The Bluetooth bridge allows controlling the robot wirelessly from a PC through an intermediate Linux device (e.g., imx8mp, Raspberry Pi).

### System Architecture

```
┌──────────────┐      Bluetooth      ┌─────────────────┐      USB/Serial      ┌─────────────┐
│   PC + GUI   │ ←─────────────────→ │  imx8mp Bridge  │ ←──────────────────→ │   Arduino   │
│ robot_test.py│    RFCOMM SPP       │  bt_bridge/*.py │    /dev/ttyACM0      │  Motor Ctrl │
└──────────────┘                     └─────────────────┘                      └─────────────┘
```

### Bridge Device Setup (imx8mp/Linux SBC)

#### 1. Copy Bridge Files

```bash
# On the bridge device
mkdir -p /opt/bt_arduino_bridge
scp tools/bt_bridge/*.py user@bridge-device:/opt/bt_arduino_bridge/
```

#### 2. Install Dependencies

```bash
# On the bridge device
apt-get update
apt-get install -y python3 python3-serial bluez rfcomm
pip3 install pyserial
```

#### 3. Start the Bridge Service

```bash
# Manual start
python3 /opt/bt_arduino_bridge/bt_arduino_bridge.py

# Or install as systemd service
cat > /etc/systemd/system/bt-arduino-bridge.service << EOF
[Unit]
Description=Bluetooth Arduino Bridge
After=bluetooth.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 /opt/bt_arduino_bridge/bt_arduino_bridge.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl enable bt-arduino-bridge
systemctl start bt-arduino-bridge
```

### Bridge Service Details

The bridge service (`bt_arduino_bridge.py`) performs:

1. Initializes Bluetooth adapter (`hciconfig hci0 up`)
2. Sets device name to "Arduino_BT_Bridge"
3. Makes device discoverable and pairable
4. Registers Serial Port Profile (SPP) on RFCOMM channel 1
5. Listens for incoming Bluetooth connections
6. Spawns `handle_connection.py` for each connection

### Per-Connection Handler

The handler (`handle_connection.py`) provides bidirectional relay:

```
Bluetooth RX → Forward to Arduino Serial TX
Arduino Serial RX → Forward to Bluetooth TX
```

Features:
- Non-blocking I/O using `select()`
- Echo filtering (removes command echo from Arduino)
- Detailed logging to `/tmp/bt_bridge_*.log`

### Connecting from PC

1. **Pair with the bridge device**
   ```bash
   # On PC
   bluetoothctl
   > scan on
   > pair XX:XX:XX:XX:XX:XX  # Bridge device address
   > trust XX:XX:XX:XX:XX:XX
   > quit
   ```

2. **Use the GUI**
   - Launch `python3 tools/robot_test.py`
   - Click "Scan Bluetooth"
   - Select "Arduino_BT_Bridge" from the list
   - Click "Connect"

3. **Or connect manually**
   ```bash
   # Create RFCOMM device
   sudo rfcomm bind 0 XX:XX:XX:XX:XX:XX 1
   # Now /dev/rfcomm0 acts like a serial port
   ```

### Bridge Logs (on imx8mp)

```bash
# Service log (connection events)
cat /tmp/bt_arduino_bridge.log

# Command log (all TX/RX with timestamps)
cat /tmp/bt_bridge_commands.log

# Debug log (detailed)
cat /tmp/bt_bridge_debug.log
```

### Troubleshooting Bluetooth

```bash
# Check Bluetooth status
hciconfig hci0

# Check if bridge is listening
rfcomm -a

# Restart Bluetooth
systemctl restart bluetooth
hciconfig hci0 reset
hciconfig hci0 up

# Check for blocked Bluetooth
rfkill list
rfkill unblock bluetooth
```

---

## PS2 Controller Support

### Connection

The PS2 controller is optional and auto-detected at startup.

### Button Mapping

| Button | Action |
|--------|--------|
| **D-pad Up** | Forward |
| **D-pad Down** | Backward |
| **D-pad Left** | Rotate Left (CCW) |
| **D-pad Right** | Rotate Right (CW) |
| **L1** | Strafe Left |
| **R1** | Strafe Right |
| **L2 + D-pad Up** | Diagonal Forward-Left |
| **L2 + D-pad Down** | Diagonal Backward-Left |
| **R2 + D-pad Up** | Diagonal Forward-Right |
| **R2 + D-pad Down** | Diagonal Backward-Right |
| **Triangle** | Fast Speed (100) |
| **X (Cross)** | Slow Speed (30) |
| **No modifier** | Normal Speed (50) |

### Behavior

- PS2 operates in **velocity mode** (continuous movement while button held)
- Releasing buttons causes motors to **coast** (no brake)
- PS2 input has priority over serial commands
- Auto-reconnects if controller disconnects

---

## Motor Control System

### Motor States

| State | IN1 | IN2 | Effect |
|-------|-----|-----|--------|
| **Forward** | PWM | 0V | Motor spins forward |
| **Backward** | 0V | PWM | Motor spins backward |
| **Brake** | 5V | 5V | Short-circuit brake (quick stop) |
| **Coast** | 0V | 0V | Motor freewheels (no power) |

### Idle State Behavior

When the robot is idle (no movement command active):
- All motors are in **Coast** mode
- **IN1 = 0, IN2 = 0** for all motors
- **No current flows** through motors
- Motors can spin freely (no holding torque)

### After Movement Completes

1. Target reached (encoder feedback)
2. **Active brake** applied for 40ms (IN1=HIGH, IN2=HIGH)
3. **Release to coast** (IN1=LOW, IN2=LOW)
4. State returns to IDLE

### Gain Compensation

Each motor has a gain factor (0.5-1.5) to compensate for mechanical differences:

```cpp
actualPWM = requestedSpeed * gain[motor]
```

Gains are computed during calibration and normalized so the fastest motor has gain = 1.0.

---

## Encoder System

### Specifications

- **Type**: Quadrature encoder
- **Resolution**: 4320 counts per revolution (CPR)
- **Interface**: Pin change interrupts (QGPMaker library)

### Reading Encoders

The `READ` command returns 4 encoder values:

```
< READ
> 1234    # Front-Left encoder
> 1230    # Front-Right encoder
> 1228    # Rear-Left encoder
> 1232    # Rear-Right encoder
```

### Encoder Reset

Encoders are automatically reset to 0 at the start of each movement command.

---

## Calibration

### Purpose

Calibration measures each motor's actual travel distance and computes gain factors so all wheels travel equally for balanced movement.

### Running Calibration

```bash
# Via serial
SYNC

# Via GUI
Click the "SYNC" button
```

### Calibration Process

1. Robot performs 6 forward movement sessions
2. Each session: move at speed 120 for 3000 encoder ticks
3. Measure actual encoder counts for each motor
4. Calculate gain correction: `gain = reference / actual_ticks`
5. Normalize gains (fastest motor = 1.0)
6. Apply gains to motor driver

### Duration

Calibration takes approximately **10 seconds** and is **blocking** (no other commands accepted).

---

## Testing Guide

### 1. Basic Communication Test

```bash
# Connect via serial monitor
pio device monitor

# Send READ command
READ

# Expected: 4 lines of encoder values (should be 0 if motors haven't moved)
```

### 2. Motor Test

```bash
# Test forward movement
FWD,50,1000

# Expected: Motors spin, robot moves forward, then "DONE" response
```

### 3. Encoder Test

```bash
# Reset and read
READ          # Note initial values
FWD,50,1000   # Move
READ          # Values should have increased
```

### 4. Calibration Test

```bash
# Run calibration
SYNC

# Expected: Robot moves forward multiple times, then "DONE"
```

### 5. GUI Test

```bash
python3 tools/robot_test.py
```

1. Connect to Arduino (USB or Bluetooth)
2. Click directional buttons
3. Verify motors respond
4. Check encoder values with READ button

### 6. Bluetooth Bridge Test

On the bridge device (imx8mp):
```bash
# Start bridge
python3 /opt/bt_arduino_bridge/bt_arduino_bridge.py

# Check logs
tail -f /tmp/bt_arduino_bridge.log
```

On PC:
```bash
# Scan and connect
python3 tools/robot_test.py
# Select Bluetooth, scan, connect to "Arduino_BT_Bridge"
# Send commands and verify response
```

---

## Troubleshooting

### Arduino Not Responding

1. Check USB connection
2. Verify correct port selected
3. Check baud rate is 9600
4. Reset Arduino and wait for "READY"

### Motors Not Moving

1. Check motor power supply
2. Verify PCA9685 I2C address (0x60)
3. Check motor wiring to correct channels
4. Run `READ` to verify encoder feedback

### Bluetooth Connection Issues

1. Ensure Bluetooth is unblocked: `rfkill unblock bluetooth`
2. Check bridge service is running
3. Verify device is paired and trusted
4. Check logs: `cat /tmp/bt_arduino_bridge.log`

### Erratic Motor Behavior

1. Run calibration: `SYNC`
2. Check encoder connections
3. Verify adequate motor power supply

### GUI Not Starting

1. Install dependencies: `pip3 install pyserial tkinter`
2. For Bluetooth: `pip3 install pybluez`
3. Check Python version: `python3 --version` (need 3.6+)

---

## Build Configuration

### platformio.ini

```ini
[env:uno]
platform = atmelavr
board = uno
framework = arduino
build_flags =
    -DTWI_FREQ=400000    # I2C fast mode
    -I src/core
    -I src/hal
    -I src/app
monitor_speed = 9600
```

### Configuration Constants (src/core/config.h)

| Constant | Value | Description |
|----------|-------|-------------|
| `SERIAL_BAUD` | 9600 | Serial communication speed |
| `PCA9685_I2C_ADDR` | 0x60 | Motor shield I2C address |
| `PCA9685_PWM_FREQ` | 1600 | PWM frequency for motors |
| `CONTROL_FREQ_HZ` | 50 | Control loop frequency |
| `ENCODER_CPR` | 4320 | Encoder counts per revolution |
| `SPEED_MIN` | 20 | Minimum motor speed |
| `SPEED_MAX` | 255 | Maximum motor speed |
| `BRAKE_HOLD_MS` | 40 | Brake duration after movement |
| `MOVE_TIMEOUT_MS` | 30000 | Movement timeout (30s) |

---

## License

MIT License
