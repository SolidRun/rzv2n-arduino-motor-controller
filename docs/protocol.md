# Serial Command Protocol

## Connection Settings

| Setting | Value |
|---------|-------|
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Line ending | `\n` (newline) or `\r\n` |
| Command format | `COMMAND,param1,param2\n` |
| Buffer size | 32 bytes max per command |
| Encoding | ASCII text |

---

## Movement Commands (Position Mode)

Move a fixed distance, then stop automatically. Speed: 20-255. Ticks: encoder counts.

| Command | Format | Description | Example |
|---------|--------|-------------|---------|
| `FWD` | `FWD,speed,ticks` | Drive forward | `FWD,100,1719` (~10cm) |
| `BWD` | `BWD,speed,ticks` | Drive backward | `BWD,80,860` (~5cm) |
| `LEFT` | `LEFT,speed,ticks` | Strafe left | `LEFT,80,1719` |
| `RIGHT` | `RIGHT,speed,ticks` | Strafe right | `RIGHT,80,1719` |
| `TURN` | `TURN,speed,ticks` | Rotate (positive ticks=CCW, negative=CW) | `TURN,60,500` (CCW) or `TURN,60,-500` (CW) |
| `DIAGFL` | `DIAGFL,speed,ticks` | Diagonal forward-left | `DIAGFL,80,1000` |
| `DIAGFR` | `DIAGFR,speed,ticks` | Diagonal forward-right | `DIAGFR,80,1000` |
| `DIAGBL` | `DIAGBL,speed,ticks` | Diagonal backward-left | `DIAGBL,80,1000` |
| `DIAGBR` | `DIAGBR,speed,ticks` | Diagonal backward-right | `DIAGBR,80,1000` |

> **Preemption:** Sending a new movement command while MOVING replaces the current motion immediately. The old command gets a `DONE` response before the new one starts.

**Distance conversion:** `ticks = distance_mm x 17.19` (4320 CPR / (pi x 80mm wheel))

## Control Commands

| Command | Format | Description | Response |
|---------|--------|-------------|----------|
| `STOP` | `STOP` | Emergency stop. Brakes all motors, cancels any operation. **Always safe to send in any state.** | `DONE` |
| `READ` | `READ` | Read all 4 encoder values. Works in any state without interrupting. | Four int32 values, one per line |
| `VEL` | `VEL,vx,vy,wz` | Continuous velocity (see [Velocity Mode](#velocity-mode)) | `OK` (first time), then `ODOM,...` at 20Hz |
| `CALIB` | `CALIB` | Run motor calibration (~40 sec): dead-zone + forward + reverse | `CALIB,...` progress, then `DONE` |

## Diagnostic Commands

| Command | Format | Description | Response |
|---------|--------|-------------|----------|
| `TMOTOR` | `TMOTOR,motor,pwm` | Drive one motor with raw PWM (no PID). Motor: `0-3` or `FL`/`FR`/`RL`/`RR`. PWM: -255 to 255. | `TEST,FL,pwm:150,FL:xxx,...` at 20Hz |
| `TENC` | `TENC` | Stream all encoders at 20Hz. No motors driven. Spin wheels by hand to test. | `ENC,...` at 20Hz |

---

## Response Messages

| Message | Meaning | When |
|---------|---------|------|
| `READY` | Arduino booted and initialized | Once at startup |
| `Robot initialized` | All subsystems ready | After READY |
| `OK` | Command accepted, execution started | After MOVE or first VEL |
| `DONE` | Operation completed successfully | After movement finishes or STOP |
| `BUSY` | Cannot process — robot is busy | Sending MOVE during CALIBRATING, TESTING, or ERROR |
| `ERROR: Stalled` | A wheel got stuck (no progress for 5s) | During MOVING |
| `ERROR: Timeout` | Movement exceeded 30 seconds | During MOVING |
| `ERROR: Watchdog` | No VEL command received within 200ms | During VEL mode |
| `ERROR: I2C fault` | PCA9685 communication failed 5 times | Hardware problem |
| `ERROR: Buffer overflow` | Command longer than 32 characters | Malformed input |
| `ERROR: Unknown` | Unrecognized command | Typo or wrong syntax |
| `STALL,FL,pos=xxx` | Which motor stalled and where | During MOVING |
| `ENC,FL:n,FR:n,RL:n,RR:n,t_us:n` | Encoder telemetry | 1Hz idle, 20Hz moving |
| `ODOM,vx,vy,wz` | Robot velocity (mm/s, mm/s, mrad/s) | 20Hz in VEL mode |
| `CALIB,start,phase:deadzone` | Calibration begun, starting dead-zone scan | Start of CALIB |
| `CALIB,dz,FL:45` | Dead-zone found for motor (PWM value) | During dead-zone phase |
| `CALIB,phase:forward` | Starting forward speed measurement | After dead-zone |
| `CALIB,fwd,N/3,FL:n,...` | Forward session N result (ticks/period) | During forward phase |
| `CALIB,fwd,done,FL:n,...` | Final forward max tick rates | Forward complete |
| `CALIB,phase:reverse` | Starting reverse speed measurement | After forward |
| `CALIB,rev,N/3,FL:n,...` | Reverse session N result (ticks/period) | During reverse phase |
| `CALIB,rev,done,FL:n,...` | Final reverse max tick rates | Reverse complete |
| `CALIB,saved` | All data written to EEPROM (v2 format) | After all phases |
| `CALIB,loaded,fwd,FL:n,...` | Forward data loaded from EEPROM | At boot |
| `CALIB,loaded,rev,FL:n,...` | Reverse data loaded from EEPROM | At boot |
| `CALIB,loaded,dz,FL:n,...` | Dead-zone data loaded from EEPROM | At boot |
| `CALIB,aborted` | Calibration stopped by STOP | During CALIB |
| `Moving: remain=n` | Debug: ticks remaining | 1Hz during position move |
| `ENC_RESET` | Encoders zeroed | At boot, before new motion, and during TMOTOR/TENC |
| `TMOTOR,FL,pwm:N` | Acknowledge test motor command | After TMOTOR command |
| `TENC,started` | Encoder test started | After TENC command |

---

## Velocity Mode

### How It Works

`VEL,vx,vy,wz` sets continuous motor velocities using mecanum inverse kinematics. Unlike position commands, VEL mode runs **forever** until you send STOP or the watchdog triggers.

| Parameter | Range | Meaning | Unit |
|-----------|-------|---------|------|
| vx | -255 to 255 | Forward (+) / Backward (-) | PWM scale |
| vy | -255 to 255 | Left strafe (+) / Right strafe (-) | PWM scale |
| wz | -255 to 255 | Counter-clockwise (+) / Clockwise (-) | PWM scale |

### Internal Processing

1. `VEL,vx,vy,wz` is parsed by `serial_cmd.cpp`
2. `robot.cpp` calls `Mecanum::computeFromVelocity(vx, vy, wz, motorSpeeds)` — this computes per-motor speeds using the standard mecanum formula with normalization
3. `Motion::setMotorVelocities(motorSpeeds)` converts PWM-scale speeds to tick-rate targets: `velSetpoint[i] = speeds[i] x calMaxTickrate[i] / 255`
4. Every 20ms, `Motion::update()` runs the PID for each motor against its `velSetpoint`
5. Every 50ms, `robot.cpp` computes encoder deltas -> forward kinematics -> sends `ODOM,vx_mm,vy_mm,wz_mrad`

### Key Behaviors

- **Watchdog:** The PC must send VEL commands at least every 200ms. If commands stop (PC crashed, cable disconnected), the robot auto-stops and sends `ERROR: Watchdog`. The GUI sends VEL at 10Hz (every 100ms), well within the 200ms window.
- **PID controlled:** Motor speeds are not just "set and forget" PWM — each motor has a PID loop tracking its target speed using encoder feedback.
- **Preemption:** Each new VEL command immediately replaces the previous velocity targets. No need to send STOP between VEL commands.
- **First VEL initializes PID and ODOM:** On the first VEL command (transition from non-VEL mode), two things happen:
  1. **PID initialization:** `motion.cpp` reads the current encoder positions and sets each motor's `prevTicks` to the current value. This prevents the PID from seeing a huge stale delta on the first tick (which would cause a PWM spike). The integral accumulators are also zeroed.
  2. **ODOM baseline:** `robot.cpp` records the current encoder positions as the ODOM delta baseline and sends `OK`.
  Subsequent VEL commands update `velSetpoint` without resetting the PID (to maintain smooth tracking). They also don't get individual `OK` responses to avoid flooding the serial line.
- **ODOM output:** `ODOM,vx,vy,wz` reports actual measured robot velocity in real-world units: mm/s for linear, mrad/s for angular. This can be consumed by ROS2 for localization.

### Examples

```bash
VEL,100,0,0      # Go forward at speed 100
VEL,0,80,0       # Strafe left at speed 80
VEL,0,0,50       # Rotate counter-clockwise at speed 50
VEL,100,50,0     # Forward + left strafe = diagonal
VEL,80,0,30      # Forward while turning = arc
VEL,0,0,0        # Stop (but watchdog timer still running — better to use STOP)
STOP              # Full stop, exit VEL mode, disable watchdog
```

---

## Parser Internals

The parser in `serial_cmd.cpp` is designed for minimal RAM usage on the 2KB ATmega328P:

1. **No `String` class** — uses a fixed 32-byte `char[]` buffer. The Arduino `String` class fragments the heap and can crash on low-memory MCUs.
2. **Character-by-character reading** — `update()` reads one byte at a time from `Serial.available()` and appends to the buffer. When `\n` or `\r` is received, it null-terminates and calls `parse()`.
3. **Prefix matching** — `match()` compares the buffer against command names character by character (not `strcmp`, to avoid reading past the comma delimiter).
4. **Lookup table** — movement commands use a static array of `{name, Direction}` pairs. To add a new direction, you add one line to the table.
5. **Overflow protection** — if the buffer is full and more characters arrive, an overflow flag is set. The entire command is discarded and `ERROR: Buffer overflow` is sent when `\n` arrives.

---

## Full Session Example

```
< (PC sends)     > (Arduino responds)

                  > CALIB,loaded,fwd,FL:153,RL:150,RR:155,FR:159
                  > CALIB,loaded,rev,FL:150,RL:148,RR:153,FR:157
                  > CALIB,loaded,dz,FL:45,RL:42,RR:48,FR:44
                  > READY
                  > Robot initialized
                  > ENC,FL:0,FR:0,RL:0,RR:0,t_us:12345       # 1Hz idle telemetry

< FWD,100,1719
                  > MOVE cmd: dir=1 spd=100 ticks=1719
                  > OK
                  > ENC,FL:200,FR:195,RL:202,RR:198,t_us:...   # 20Hz telemetry
                  > ENC,FL:500,FR:495,RL:502,RR:498,t_us:...
                  > Moving: remain=1219                          # 1Hz debug
                  > ...
                  > DONE                                         # Target reached

< VEL,100,0,0
                  > OK
                  > ODOM,145,3,-2                                # 20Hz odometry
                  > ODOM,146,2,-1
< VEL,100,50,0                                                   # Change direction
                  > ODOM,140,68,-3                                # Now strafing too
< STOP
                  > DONE

< TMOTOR,FL,150
                  > TMOTOR,FL,pwm:150
                  > TEST,FL,pwm:150,FL:250,FR:0,RL:0,RR:0       # Only FL moves
< STOP
                  > DONE

< CALIB
                  > CALIB,start,phase:deadzone
                  > CALIB,dz,FL:45
                  > CALIB,dz,RL:42
                  > CALIB,dz,RR:48
                  > CALIB,dz,FR:44
                  > CALIB,phase:forward
                  > CALIB,fwd,1/3,FL:120,RL:118,RR:122,FR:125
                  > CALIB,fwd,2/3,FL:121,RL:117,RR:123,FR:124
                  > CALIB,fwd,3/3,FL:120,RL:118,RR:122,FR:125
                  > CALIB,fwd,done,FL:153,RL:150,RR:155,FR:159
                  > CALIB,phase:reverse
                  > CALIB,rev,1/3,FL:118,RL:116,RR:120,FR:123
                  > CALIB,rev,2/3,FL:119,RL:115,RR:121,FR:122
                  > CALIB,rev,3/3,FL:118,RL:116,RR:120,FR:123
                  > CALIB,rev,done,FL:150,RL:148,RR:153,FR:157
                  > CALIB,saved
                  > DONE
```
