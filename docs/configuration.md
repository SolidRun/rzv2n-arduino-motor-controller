# Configuration Reference

All constants are in `src/core/config.h`. Changing a value here automatically propagates to all code that uses it.

## Build Flags

| Flag | Default | Description |
|------|---------|-------------|
| `DEBUG_ENABLED` | Defined | Enables `DBG_PRINT`/`DBG_PRINTLN` macros for state transitions |
| `DEBUG_TIMING` | Commented out | Would enable timing measurements |
| `DEBUG_MOTORS` | Commented out | Would enable per-motor debug output |
| `ENABLE_PS2` | Commented out | Enable PS2 controller support (~3KB Flash) |

## Hardware Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `PCA9685_I2C_ADDR` | `0x60` | PWM driver I2C address |
| `PCA9685_PWM_FREQ` | `1600` | PWM frequency (Hz) |
| `SERIAL_BAUD` | `115200` | Serial communication speed |
| `CMD_BUFFER_SIZE` | `32` | Max serial command length (bytes) |
| `NUM_MOTORS` | `4` | Number of motors |
| `NUM_ENCODERS` | `4` | Number of encoders |
| `ENCODER_CPR` | `4320` | Encoder counts per revolution |
| `WHEEL_DIAMETER_MM` | `80` | Wheel diameter (mm) |
| `WHEEL_BASE_MM` | `190` | Front-to-rear axle distance (mm) |
| `TRACK_WIDTH_MM` | `210` | Left-to-right wheel distance (mm) |

## Motor Channel Mapping

| Constant | Value | Description |
|----------|-------|-------------|
| `MOTOR_FL` / `MOTOR_RL` / `MOTOR_RR` / `MOTOR_FR` | `0 / 1 / 2 / 3` | Motor array indices |
| `FL_IN1_CH` / `FL_IN2_CH` | `8 / 9` | Front-Left PCA9685 channels |
| `RL_IN1_CH` / `RL_IN2_CH` | `10 / 11` | Rear-Left channels |
| `RR_IN1_CH` / `RR_IN2_CH` | `15 / 14` | Rear-Right channels |
| `FR_IN1_CH` / `FR_IN2_CH` | `13 / 12` | Front-Right channels |
| `FL_ENC_PORT` / `RL_ENC_PORT` / `RR_ENC_PORT` / `FR_ENC_PORT` | `1 / 2 / 3 / 4` | Encoder ports |
| `FL_ENC_DIR` / `RL_ENC_DIR` / `RR_ENC_DIR` / `FR_ENC_DIR` | `+1 / +1 / -1 / -1` | Encoder direction multipliers |

## Control Loop

| Constant | Value | Description |
|----------|-------|-------------|
| `CONTROL_FREQ_HZ` | `50` | PID update rate |
| `CONTROL_PERIOD_US` | `20000` | Period in microseconds (20ms) |
| `TIMER1_PRESCALER` | `256` | Timer1 clock divider |
| `TIMER1_OCR_VALUE` | `1249` | Compare match value for 50Hz |
| `SPEED_MIN` / `SPEED_MAX` | `20 / 255` | Accepted speed range |

## PID Parameters

| Constant | Value | Description |
|----------|-------|-------------|
| `VEL_PID_KP` | `1.5` | Proportional gain (PWM per tick/period error) |
| `VEL_PID_KI` | `0.3` | Integral gain (eliminates steady-state error) |
| `VEL_PID_IMAX` | `150.0` | Anti-windup integral limit |
| `MAX_MOTOR_TICKRATE` | `150` | Default max ticks per period (before calibration) |

## Motion Profile

| Constant | Value | Description |
|----------|-------|-------------|
| `ACCEL_RAMP_TICKS` | `300` | Accelerate over first N ticks (~1.7cm) |
| `ACCEL_MIN_SPEED` | `40` | Starting speed during acceleration |
| `SLOWDOWN_TICKS` | `400` | Start decelerating within N ticks of target (~2.3cm) |
| `SLOWDOWN_KP` | `0.05` | Deceleration gain (speed per remaining tick) |
| `SLOWDOWN_MIN_SPEED` | `60` | Minimum speed during deceleration |
| `MIN_WORK_SPEED` | `80` | Minimum speed for short moves (<=250 ticks) |
| `SHORT_MOVE_TICKS` | `250` | Threshold for short move handling (~1.5cm) |
| `SHORT_REMAIN_TICKS` | `600` | Defined but **unused in code** (reserved for future use) |
| `BRAKE_HOLD_MS` | `40` | Active brake duration after movement |

## Safety Timeouts

| Constant | Value | Description |
|----------|-------|-------------|
| `MOVE_TIMEOUT_MS` | `30000` | Max time for any position move (30s) |
| `WATCHDOG_TIMEOUT_MS` | `200` | VEL mode auto-stop if no command |
| `STALL_TIMEOUT_MS` | `5000` | No encoder progress -> stall (5s) |
| `STALL_MIN_PROGRESS` | `50` | Min ticks to reset stall timer |
| `I2C_HEALTH_CHECK_MS` | `2000` | I2C ping interval (2s) |
| `I2C_ERROR_THRESHOLD` | `5` | Consecutive I2C failures -> ERROR |
| `ERROR_RECOVERY_MS` | `5000` | Auto-recover from ERROR (5s) |

## Calibration

| Constant | Value | Description |
|----------|-------|-------------|
| `CALIB_PWM` | `200` | Reference PWM for measurement |
| `CALIB_SETTLE_MS` | `500` | Wait for steady state |
| `CALIB_MEASURE_MS` | `2000` | Measurement window per session |
| `CALIB_SESSIONS` | `3` | Number of sessions (averaged) |
| `CALIB_BRAKE_MS` | `300` | Brake between sessions |
| `EEPROM_CALIB_ADDR` | `0` | EEPROM start address |
| `EEPROM_CALIB_MARKER` | `0xCB` | Validity marker byte |

## Telemetry Rates

| Constant | Value | Description |
|----------|-------|-------------|
| `IDLE_TELEMETRY_MS` | `1000` | Encoder print rate in IDLE (1Hz) |
| `MOVING_TELEMETRY_MS` | `50` | Encoder/ODOM print rate while MOVING (20Hz) |
| `MOVING_DEBUG_MS` | `1000` | "Moving: remain=N" debug output (1Hz) |

## PS2 Controller

| Constant | Value | Description |
|----------|-------|-------------|
| `PS2_DAT_PIN` / `PS2_CMD_PIN` / `PS2_ATT_PIN` / `PS2_CLK_PIN` | `12 / 11 / 10 / 13` | Controller pins |
| `PS2_READ_MS` | `30` | Poll interval (33Hz) |
| `PS2_RECONNECT_MS` | `1000` | Reconnect attempt interval |
| `PS2_DISCONNECT_MS` | `1000` | Disconnect detection threshold |
| `PS2_SPEED_SLOW` / `PS2_SPEED_NORMAL` / `PS2_SPEED_FAST` | `30 / 50 / 100` | Speed presets |

---

# Safety Systems

## Overview

| Protection | What It Detects | Response | Config |
|-----------|----------------|----------|--------|
| **Stall detection** | A single wheel stuck | Coast all, `ERROR: Stalled`, back to IDLE | `STALL_TIMEOUT_MS=5000`, `STALL_MIN_PROGRESS=50` |
| **Move timeout** | Movement takes too long | Coast all, `ERROR: Timeout`, back to IDLE | `MOVE_TIMEOUT_MS=30000` |
| **Watchdog** (VEL only) | PC crashed or cable disconnected | Coast all, `ERROR: Watchdog`, back to IDLE | `WATCHDOG_TIMEOUT_MS=200` |
| **I2C health** | PCA9685 communication failure | Coast all, enter ERROR state, auto-recover | `I2C_ERROR_THRESHOLD=5` |
| **I2C bus timeout** | I2C bus hung (SDA stuck low) | Wire library resets the bus | `Wire.setWireTimeout(3000, true)` |
| **Buffer overflow** | Command > 32 chars | Discard command, `ERROR: Buffer overflow` | `CMD_BUFFER_SIZE=32` |
| **Calibration timeout** | Calibration hung | Abort calibration, ERROR | 30s hard limit |
| **ERROR auto-recovery** | Any ERROR state | Auto-transitions back to IDLE | `ERROR_RECOVERY_MS=5000` |

## Stall Detection Details

Stall detection is **per-motor, not global**. Each active motor is individually tracked:

```c
for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (!(activeMask & (1 << i))) continue;  // Skip inactive motors

    int32_t absPos = abs(snap.ticks[i]);
    if (absPos - lastProgressPos[i] >= STALL_MIN_PROGRESS) {  // 50 ticks
        lastProgressPos[i] = absPos;
        lastProgressTime[i] = now;
    } else if (now - lastProgressTime[i] > STALL_TIMEOUT_MS) {  // 5000ms
        // No progress for 5 seconds — this motor is stalled
        Serial.print(F("STALL,")); Serial.print(motorName);
        Serial.print(F(",pos=")); Serial.println(absPos);
        stalled = true;
        target.active = false;
        Motor::coastAll();
        return;
    }
}
```

The stall message includes the motor name (FL/RL/RR/FR) and its position.

## Watchdog Details

The watchdog only applies to VEL mode. Position moves have their own timeout (`MOVE_TIMEOUT_MS`).

- Every command resets the watchdog timer: `lastCommandTime = millis()`
- Enabled when entering VEL mode, disabled for position moves and calibration
- Sending `STOP` resets the watchdog timer and transitions to IDLE (watchdog check only runs in MOVING state)

---

# Troubleshooting

## Arduino Not Responding

1. Check USB cable (some cables are charge-only, no data)
2. Verify correct serial port: `/dev/ttyACM0` (Linux), `/dev/cu.usbmodem*` (Mac), `COM3` (Windows)
3. Verify baud rate is 115200
4. Press the reset button on Arduino and wait for `READY` message
5. If the serial port doesn't appear on Linux:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and log back in
   ```

## Motors Don't Move

1. **Check power:** Motors need external power (battery), not just USB.
2. **Verify PCA9685:** Upload hwtest firmware, run `I2C` — should see `0x60 (PCA9685)`.
3. **Test individual motor:** `TMOTOR,FL,150` — if no motor moves, it's a PCA9685 or power problem.
4. **Check channel wiring:** Run `MAP` in hwtest firmware.

## Wrong Motor Responds

The motor-to-channel wiring doesn't match `config.h`. Use `TMOTOR,FL,100` and watch which physical wheel spins. Adjust `FL_IN1_CH` / `FL_IN2_CH` in `config.h`.

## Motor Spins Wrong Direction

Swap the `IN1` and `IN2` channel numbers for that motor in `config.h`:
```c
// Before (wrong):
#define FL_IN1_CH  8
#define FL_IN2_CH  9

// After (swapped):
#define FL_IN1_CH  9
#define FL_IN2_CH  8
```

## Robot Drifts or Drives Crooked

1. **Run calibration:** `CALIB`
2. **Check encoder directions:** `TENC`, spin each wheel forward by hand — should increase. If decreases, flip `*_ENC_DIR`.
3. **Check encoder-motor pairing:** `TMOTOR,FL,100` — only FL encoder should move.

## Stall Errors

1. Verify wheels spin freely
2. Check motor power supply voltage
3. Try increasing `STALL_TIMEOUT_MS` (e.g., 10000)
4. Ensure encoders working: `TENC`, spin wheel — if 0, encoder disconnected

## "ERROR: I2C fault"

1. Check SDA (A4) and SCL (A5) connections
2. Check for loose wires or cold solder joints
3. Run `I2C` scan in hwtest firmware
4. Long wires (>30cm): add 4.7k pull-up resistors on SDA and SCL

## "ERROR: Watchdog" in VEL Mode

PC stopped sending VEL commands for >200ms:
1. Serial port disconnected
2. Python script crashed
3. PC too slow — ensure VEL sent at >= 5Hz

## GUI Won't Start

```bash
sudo apt install python3-tk    # Linux only
cd tools/
rm -rf venv
./install.sh
python3 run.py
```

## Build Errors

```bash
# PlatformIO not found
pip install platformio

# Permission denied on serial port (Linux)
sudo usermod -a -G dialout $USER

# Multiple boards — specify port
pio run -t upload --upload-port /dev/ttyACM0
```
