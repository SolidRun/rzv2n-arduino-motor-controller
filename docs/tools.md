# Tools & Controllers

## GUI Controller (`tools/robot_test.py`)

A full-featured graphical interface for controlling the robot over USB serial.

### Layout

```
┌─ Connection ──────────────────────────────────────────────┐
│  Port: [▼]  [Refresh]  [Connect]   Status   State        │
├─ Live Telemetry (always visible) ─────────────────────────┤
│  ENC  FL: ---  FR: ---  RL: ---  RR: ---  │  ODOM  vx: --- vy: --- wz: --- │
├───────────────────────────────────────────────────────────┤
│ [ Position ] [ Velocity ] [ Motor Test ] [ Smart Calibration ] │
│                                                           │
│  (selected tab content)                                   │
│                                                           │
├─ Serial Monitor (always visible) ─────────────────────────┤
│  12:34:56 -> FWD,100,1000                                 │
│  12:34:56 <- OK                                           │
└───────────────────────────────────────────────────────────┘
```

### Tabs

- **Position** — speed and ticks number fields, 3x3 directional grid, CCW/CW rotation, Read Encoders
- **Velocity** — vx/vy/wz number fields (-255 to 255), Start/Stop buttons, streams VEL commands at 10Hz
- **Motor Test** — motor selector (FL/FR/RL/RR), PWM number field, Run Motor, Test Encoders, Quick/Smart Calibrate
- **Smart Calibration** — full motor calibration (dead-zone + fwd/rev speed), ticks-per-mm measurement with quick-test presets and results table, export JSON / update config.h

### Always Visible

- **Live Telemetry** — compact bar showing encoder ticks (FL/FR/RL/RR) and odometry (vx/vy/wz) from serial data, updates in real-time regardless of which tab is selected
- **Serial Monitor** — dark-themed console with color-coded messages (blue=TX, green=RX, bold green=OK/DONE, red=ERROR), timestamp prefixes, manual command entry, auto-scroll and telemetry filter toggles

### Keyboard Shortcuts

| Key | Action | Key | Action |
|-----|--------|-----|--------|
| W | Forward | Q | Rotate CCW |
| S | Backward | E | Rotate CW |
| A | Strafe Left | Space | STOP |
| D | Strafe Right | R | Read Encoders |
| 7 | Diagonal FL | 9 | Diagonal FR |
| 1 | Diagonal BL | 3 | Diagonal BR |

### Setup and Launch

```bash
cd tools/
python3 install.py       # Create venv + install pyserial (cross-platform)
python3 run.py           # Launch GUI (checks venv exists)

# Or directly:
./venv/bin/python robot_test.py
```

---

## Hardware Test CLI (`tools/hw_test.py`)

An interactive terminal-based tool for the hardware test firmware.

```bash
python3 tools/hw_test.py             # Auto-detect port
python3 tools/hw_test.py /dev/ttyACM0  # Specify port
```

### Commands

| CLI Command | What It Does |
|-------------|-------------|
| `m <idx> <speed>` | Run motor (0=FL 1=RL 2=RR 3=FR), speed -255..255 |
| `pin <ch> <duty>` | Set PCA9685 channel, duty 0-4095 (or `on`/`off`) |
| `enc` | Read all encoders once |
| `encr` | Reset all encoders |
| `live` | Toggle live encoder stream at 10Hz (press Enter to stop) |
| `stop` | Coast all motors |
| `brake` | Brake all motors |
| `map` | Show pin mapping |
| `i2c` | Scan I2C bus |
| `sweep <idx>` | Sweep motor speed -255 -> +255 with progress bar (direction test) |
| `test all` | Briefly spin each motor forward then reverse |
| `test enc` | Reset encoders, run all motors 1s at PWM=150, print encoder counts |
| `quit` | Stop motors and exit |

---

## Hardware Test Firmware

A separate, minimal firmware for testing individual hardware components without the application layer (no PID, no state machine, no kinematics). Useful for verifying wiring after assembly or diagnosing problems.

### Build and Upload

```bash
# Build hardware test firmware
pio run -e hwtest

# Upload to Arduino (replaces main firmware!)
pio run -e hwtest -t upload

# Open serial monitor
pio device monitor -e hwtest

# To go back to normal firmware:
pio run -t upload
```

### Test Firmware Commands

| Command | Format | What It Does |
|---------|--------|-------------|
| `MOT` | `MOT,idx,speed` | Drive single motor. idx: 0-3 (FL=0,RL=1,RR=2,FR=3). Speed: -255 to 255. |
| `PIN` | `PIN,ch,duty` | Set raw PCA9685 channel 0-15 to duty cycle 0-4095. Bypasses motor abstraction. |
| `ENC` | `ENC` | Read all 4 encoder values once. |
| `ENCR` | `ENCR` | Reset all encoders to zero. |
| `STOP` | `STOP` | Coast all motors (turn off). |
| `BRK` | `BRK` | Active brake all motors (IN1=IN2=HIGH). |
| `MAP` | `MAP` | Print complete wiring map: motor -> channels -> encoder ports -> directions. |
| `I2C` | `I2C` | Scan entire I2C bus (addr 1-126) and list all responding devices. |
| `LIVE` | `LIVE` | Toggle continuous encoder streaming at 10Hz. Send again to stop. |
| `HELP` | `HELP` | Print all available commands. |

Commands are **case-insensitive** (converted to uppercase internally).

### Wiring Verification Procedure

1. Upload hwtest firmware
2. Run `MAP` to see expected wiring
3. Run `I2C` to verify PCA9685 is detected at 0x60
4. Test each motor individually:
   ```
   MOT,0,150     <- should spin Front-Left wheel forward
   MOT,0,0       <- stop
   MOT,0,-150    <- should spin Front-Left wheel backward
   MOT,0,0
   ```
   Repeat for motors 1, 2, 3. If the wrong wheel spins, fix channel mapping in `config.h`.
5. Test encoders:
   ```
   ENCR           <- reset all to zero
   LIVE           <- start streaming
   ```
   Spin each wheel forward by hand. The corresponding encoder should increase (positive). If it decreases, flip `*_ENC_DIR` in `config.h`.

---

## PS2 Controller

PS2 wireless controller support is **optional** and disabled by default to save ~3KB Flash. To enable:

1. Uncomment `#define ENABLE_PS2` in `config.h`
2. Rebuild and upload: `pio run -t upload`

### Pin Wiring

| PS2 Pin | Arduino Pin |
|---------|------------|
| CLK | 13 |
| CMD | 11 |
| ATT | 10 |
| DAT | 12 |

See reference image: [PS2 button names](images/ps2-controller-buttons.jpg)

### Button Mapping

| Button | Action | Speed |
|--------|--------|-------|
| D-pad Up | Forward | Normal (50) |
| D-pad Down | Backward | Normal (50) |
| D-pad Left | Rotate CCW | Normal (50) |
| D-pad Right | Rotate CW | Normal (50) |
| L1 | Strafe Left | Normal (50) |
| R1 | Strafe Right | Normal (50) |
| L2 + D-pad Up | Diagonal Forward-Left | Normal (50) |
| L2 + D-pad Down | Diagonal Backward-Left | Normal (50) |
| R2 + D-pad Up | Diagonal Forward-Right | Normal (50) |
| R2 + D-pad Down | Diagonal Backward-Right | Normal (50) |
| Triangle | Fast speed | 100 |
| X (Cross) | Slow speed | 30 |
| Release all | Stop (coast) | 0 |

### Behavior

- PS2 operates in **velocity mode** (continuous movement while button held)
- Releasing buttons -> coast (no brake), transitions back to IDLE
- Auto-reconnects every 1000ms if controller disconnects
- PS2 is polled every 30ms (33Hz)
- Watchdog is **disabled** during PS2 control
- Serial commands still work alongside PS2 (STOP overrides PS2)
