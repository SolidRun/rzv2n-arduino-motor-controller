# Mecanum 4WD Robot Controller

Open-source Arduino Uno firmware for a 4-wheel mecanum robot with omnidirectional motion, closed-loop PID control, auto-calibration, and a cross-platform Python GUI.

**Source code:** [github.com/SolidRun/rzv2n-arduino-motor-controller](https://github.com/SolidRun/rzv2n-arduino-motor-controller)

---

## Overview

This project provides a complete motion-control stack for a mecanum-wheeled robot platform. The firmware runs on an Arduino Uno and communicates with a host PC over USB serial using a simple text-based protocol. A Python GUI is included for interactive control, diagnostics, and calibration.

### System Architecture

```
  Host PC                   Arduino Uno               PCA9685                TB67H450 x4
  ────────                  ───────────               ───────                ───────────
                             ATmega328P
  USB Serial     ──────►    State machine  ──I2C───►  16-ch PWM   ──8ch──►  H-Bridge
  115200 baud               PID @ 50Hz     400kHz     12-bit                motor drivers
  Text commands             Kinematics                1600Hz                     │
                                 ▲                                          4x DC Motors
                            Pin-change IRQ                                  9600RPM / 90:1
                                 │                                               │
                            4x Encoders  ◄────────────────────────────  4x Mecanum Wheels
                            4320 CPR                                       80mm diameter
```

Mecanum wheels use angled rollers to produce omnidirectional motion. By controlling the speed and direction of each wheel independently, the robot can move forward, sideways, diagonally, or rotate in place without mechanical steering.

### Control Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| **Position** | Move an exact distance, then auto-stop | Autonomous navigation steps |
| **Velocity** | Continuous speed control with 20Hz odometry | ROS2 integration, teleoperation |

### Hardware Bill of Materials

| Component | Specification |
|-----------|---------------|
| MCU | Arduino Uno (ATmega328P, 16MHz, 2KB RAM, 32KB Flash) |
| PWM driver | PCA9685 at I2C address 0x60, 1600Hz output |
| Motor drivers | 4x TB67H450FNG dual H-bridge |
| Motors | 4x DC gear motors, 90:1 gearbox, quadrature encoders (4320 CPR) |
| Wheels | 80mm mecanum wheels |
| Chassis | 190mm wheelbase, 210mm track width |

---

## Wiring

### Motor-to-Controller Mapping

```
              ┌──────── FRONT ────────┐
              │                       │
         FL (index 0)           FR (index 3)
         PCA9685 CH 8, 9        PCA9685 CH 13, 12
         Encoder Port 1         Encoder Port 4

         RL (index 1)           RR (index 2)
         PCA9685 CH 10, 11      PCA9685 CH 15, 14
         Encoder Port 2         Encoder Port 3
              │                       │
              └──────── REAR ─────────┘
```

| Motor | Index | PCA9685 IN1 | PCA9685 IN2 | Encoder Port | Encoder Direction |
|-------|:-----:|:-----------:|:-----------:|:------------:|:-----------------:|
| Front-Left  | 0 | CH 8  | CH 9  | Port 1 | Normal (+1) |
| Rear-Left   | 1 | CH 10 | CH 11 | Port 2 | Normal (+1) |
| Rear-Right  | 2 | CH 15 | CH 14 | Port 3 | Inverted (-1) |
| Front-Right | 3 | CH 13 | CH 12 | Port 4 | Inverted (-1) |

Right-side encoders are inverted in firmware so that forward motion produces positive tick counts on all four wheels.

### Connections

1. Mount the motor shield onto the Arduino Uno.
2. Connect the four DC motors to the shield outputs according to the table above.
3. Plug each encoder cable into its corresponding port (1 through 4).
4. Connect the Arduino to the host PC via USB.

On Linux, serial port access requires the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
# Log out and log back in for the change to take effect.
```

---

## Building and Uploading the Firmware

### Prerequisites

**Linux:**

```bash
sudo apt install build-essential python3-dev python3-tk python3-venv
pip install platformio
```

**Windows / macOS:**

Install [Python 3.8+](https://www.python.org/downloads/) and [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation.html).

### Clone and Build

```bash
git clone https://github.com/SolidRun/rzv2n-arduino-motor-controller.git
cd rzv2n-arduino-motor-controller

# Compile the firmware
pio run

# Upload to the Arduino (port is auto-detected)
pio run -t upload
```

### Verify Connection

```bash
pio device monitor
```

The Arduino prints `READY` on boot. If you see this message, the firmware is running.

---

## Testing

Testing is divided into three stages: hardware verification, basic command testing, and GUI-based interactive testing.

### Stage 1 — Hardware Verification

A separate minimal firmware (`env:hwtest`) allows direct motor and encoder testing without PID, kinematics, or the state machine. Use this to verify wiring before running the full firmware.

```bash
# Upload the hardware-test firmware
pio run -e hwtest -t upload

# Launch the interactive test CLI
cd tools/
python3 install.py        # First time only — sets up the Python environment
python3 hw_test.py
```

The test CLI provides:

- **Individual motor control** — spin each motor and confirm direction.
- **Raw encoder readout** — verify that each encoder counts in the correct direction.
- **I2C bus scan** — confirm the PCA9685 is detected at address 0x60.

> Restore the main firmware after testing: `pio run -t upload`

### Stage 2 — Command-Line Verification

With the main firmware uploaded, open the serial monitor (`pio device monitor`) and run through these commands:

```
READ                    Expect: ENC,0,0,0,0 (encoders at zero)
TMOTOR,0,100            Spin front-left motor at PWM 100
TMOTOR,0,0              Stop it
FWD,80,860              Move forward ~5 cm
STOP                    Emergency stop
```

If the robot moves in the expected direction and the encoder values change accordingly, the system is functional.

### Stage 3 — GUI Testing

```bash
cd tools/
python3 run.py
```

The GUI provides:

| Feature | Description |
|---------|-------------|
| Direction pad | Click or use keyboard (WASD + QE) to drive |
| Velocity sliders | Real-time streaming control at 10Hz |
| Live telemetry | Encoder ticks and odometry displayed continuously |
| Motor diagnostics | Test individual motors with adjustable PWM |
| Serial monitor | Color-coded log of all sent and received messages |

**Keyboard shortcuts:**

| Key | Action |
|:---:|--------|
| W / S | Forward / Backward |
| A / D | Strafe Left / Right |
| Q / E | Rotate CCW / CW |
| 7 / 9 | Diagonal Front-Left / Front-Right |
| 1 / 3 | Diagonal Back-Left / Back-Right |
| Space | Emergency Stop |
| R | Read Encoders |

---

## Calibration

The calibration routine measures each motor's physical characteristics so the PID controller can produce accurate speed estimates from the first control cycle. Calibration data is stored in EEPROM and persists across power cycles.

### When to Calibrate

- After initial assembly
- After replacing a motor, wheel, or gearbox
- If the robot drifts during straight-line moves
- After significant battery voltage changes

### Running Calibration

Send `CALIB` over serial, or use the **Smart Calibration** tab in the GUI. The process runs automatically and takes approximately 40 seconds.

| Phase | Duration | Purpose |
|-------|:--------:|---------|
| Dead-zone detection | ~24s | Ramps PWM from 0 on each motor individually until the encoder registers movement. Records the minimum PWM required to overcome static friction. |
| Forward speed measurement | ~8s | Runs all motors at PWM 200 forward across 3 sessions. Averages the encoder tick rate and extrapolates to PWM 255. |
| Reverse speed measurement | ~8s | Same procedure in reverse. Captures direction-dependent speed differences caused by H-bridge asymmetry and mechanical factors. |

### Calibration Output

```
CALIB,dz,FL:45,RL:50,RR:42,FR:48               Dead-zone (min PWM to move)
CALIB,fwd,done,FL:153,RL:150,RR:155,FR:159      Forward max tick rate
CALIB,rev,done,FL:150,RL:148,RR:153,FR:157      Reverse max tick rate
CALIB,saved                                      Written to EEPROM
DONE
```

### How Calibration Improves Control

The PID controller uses a **feed-forward + PI** architecture. The feed-forward term provides an initial PWM estimate based on calibration data, while the PI terms correct for real-time error.

| Scenario | Feed-Forward Accuracy | PID Convergence |
|----------|:---------------------:|:---------------:|
| No calibration (defaults) | ~15% error | 5-8 ticks (100-160ms) |
| After calibration | ~2% error | 1-2 ticks (20-40ms) |

**Dead-zone enforcement** ensures the PID never outputs a PWM value inside a motor's dead-zone. If the computed output falls between 1 and the motor's dead-zone threshold, it is clamped up to the threshold. This prevents stalling and integral windup at low speeds.

**Direction-aware feed-forward** selects the forward or reverse calibration value based on the motor's current direction, ensuring accurate estimates during strafe and rotation maneuvers where motors run in opposite directions.

---

## Serial Protocol Reference

All communication uses ASCII text at **115200 baud**. Commands are terminated with a newline character (`\n`). The maximum command length is 32 bytes.

### Movement Commands (Position Mode)

| Command | Format | Description |
|---------|--------|-------------|
| Forward | `FWD,<speed>,<ticks>` | Move forward |
| Backward | `BWD,<speed>,<ticks>` | Move backward |
| Strafe left | `LEFT,<speed>,<ticks>` | Move sideways left |
| Strafe right | `RIGHT,<speed>,<ticks>` | Move sideways right |
| Rotate | `TURN,<speed>,<ticks>` | Rotate (positive = CCW, negative = CW) |
| Diagonal | `DIAGxx,<speed>,<ticks>` | `xx` = FL, FR, BL, or BR |

Parameters: `speed` = 20-255, `ticks` = encoder ticks.

**Distance conversion:** `ticks = distance_mm x 17.19` (e.g., 10cm = 1719 ticks)

### Velocity Mode

| Command | Format | Description |
|---------|--------|-------------|
| Set velocity | `VEL,<vx>,<vy>,<wz>` | Continuous motion (-255 to 255 per axis) |
| Stop | `STOP` | Stop all motors immediately |

The firmware streams `ODOM,vx,vy,wz` at 20Hz during velocity mode, reporting estimated robot velocity in mm/s and mrad/s.

### Diagnostics

| Command | Format | Response |
|---------|--------|----------|
| Read encoders | `READ` | `ENC,<fl>,<rl>,<rr>,<fr>` |
| Test motor | `TMOTOR,<index>,<pwm>` | — |
| Test encoders | `TENC` | Streaming encoder values |
| Calibrate | `CALIB` | Progress updates, then `DONE` |
| Stop | `STOP` | `DONE` |

### Response Codes

| Response | Meaning |
|----------|---------|
| `READY` | Firmware booted, awaiting commands |
| `DONE` | Command completed successfully |
| `BUSY` | A move is already in progress |
| `ERROR,<msg>` | Invalid command or hardware fault |

---

## Project Structure

```
rzv2n-arduino-motor-controller/
├── src/
│   ├── app/                        Application layer
│   │   ├── main.cpp                  Entry point, 50Hz main loop
│   │   ├── robot.cpp/h               State machine, command dispatch, telemetry
│   │   ├── motion.cpp/h              PID control, position tracking, calibration
│   │   ├── mecanum.cpp/h             Kinematics (direction to motor speeds)
│   │   └── serial_cmd.cpp/h          Serial parser (32-byte buffer)
│   ├── hal/                        Hardware abstraction layer
│   │   ├── motor.cpp/h               PWM buffering, speed-to-duty conversion
│   │   ├── encoder.cpp/h             Atomic reads, snapshot helpers
│   │   ├── pca9685.cpp/h             I2C PWM driver interface
│   │   └── timer.cpp/h               Timer1 50Hz tick generation
│   ├── core/                       Shared definitions
│   │   ├── config.h                  Pin mapping, PID gains, timeouts, geometry
│   │   └── types.h                   Enums, structs, utility functions
│   └── test/                       Hardware test firmware (env:hwtest)
│       └── main.cpp                  Minimal motor/encoder test mode
├── lib/                            Third-party libraries
│   ├── Adafruit_PWMServo/            PCA9685 I2C driver
│   ├── QGPMaker/                     Encoder library (pin-change interrupts)
│   ├── PinChangeInterrupt/           AVR pin-change interrupt support
│   └── PS2X/                         PS2 wireless controller (optional)
├── tools/                          Host-side utilities
│   ├── robot_test.py                 Tkinter GUI controller
│   ├── hw_test.py                    Hardware test CLI
│   ├── install.py                    Cross-platform dependency installer
│   ├── run.py                        GUI launcher
│   └── requirements.txt              Python dependencies
├── docs/                           Documentation
│   ├── architecture.md               Software design, state machine, timing
│   ├── protocol.md                   Full serial protocol specification
│   ├── hardware.md                   Wiring reference, PCA9685 registers
│   ├── motion-control.md             Kinematics, PID tuning, calibration
│   ├── tools.md                      GUI and CLI tool reference
│   ├── configuration.md              All config.h parameters
│   └── index.html                    Interactive visual documentation
└── platformio.ini                  Build configuration
```

### Build Environments

| Environment | Command | Description |
|-------------|---------|-------------|
| `uno` (default) | `pio run` | Full firmware with PID, kinematics, and state machine |
| `hwtest` | `pio run -e hwtest` | Minimal firmware for wiring verification |

### Build Statistics

| Resource | Usage |
|----------|-------|
| Flash | 58.5% (18,854 / 32,256 bytes) |
| RAM | 53.9% (1,103 / 2,048 bytes) |

---

## Further Reading

- [Architecture](https://github.com/SolidRun/rzv2n-arduino-motor-controller/blob/main/docs/architecture.md) — three-layer design, command flow, state machine, boot sequence
- [Protocol Specification](https://github.com/SolidRun/rzv2n-arduino-motor-controller/blob/main/docs/protocol.md) — complete serial protocol with session examples
- [Hardware Reference](https://github.com/SolidRun/rzv2n-arduino-motor-controller/blob/main/docs/hardware.md) — wiring tables, PCA9685 registers, H-bridge control logic
- [Motion Control](https://github.com/SolidRun/rzv2n-arduino-motor-controller/blob/main/docs/motion-control.md) — mecanum kinematics, PID internals, speed ramping, calibration math
- [Configuration](https://github.com/SolidRun/rzv2n-arduino-motor-controller/blob/main/docs/configuration.md) — all tunable parameters and troubleshooting
- [Interactive Documentation](https://github.com/SolidRun/rzv2n-arduino-motor-controller/blob/main/docs/index.html) — visual diagrams (open in browser)

---

**License:** MIT
