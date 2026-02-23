# Software Architecture

## Three-Layer Design

The code is organized in 3 layers. Each layer only talks to the one directly below it.

```
┌──────────────────────────────────────────────────────────────────────────┐
│                    APPLICATION LAYER (src/app/)                          │
│                                                                          │
│  robot.cpp          motion.cpp         mecanum.cpp      serial_cmd.cpp   │
│  State machine      PID controller     Direction→       Serial parser    │
│  Command dispatch   Position tracking   motor signs     Response format  │
│  Telemetry output   Velocity mode      Accel/decel      Cmd lookup table │
│  I2C health check   Calibration seq    Inverse/fwd kin  Buffer overflow  │
│  PS2 controller     Stall detection    Speed normalization               │
├──────────────────────────────────────────────────────────────────────────┤
│                    HARDWARE ABSTRACTION LAYER (src/hal/)                  │
│                                                                          │
│  motor.cpp          encoder.cpp        timer.cpp        pca9685.cpp      │
│  PWM buffer batch   Atomic snapshot    Timer1 CTC ISR   I2C register     │
│  Speed→duty conv    Direction multiply  50Hz flag        write/read      │
│  Brake/coast/set    Read/reset/allReached consumeTick    Batch channels  │
│  Non-blocking brake  Average/min/max   Overrun count    Health check     │
├──────────────────────────────────────────────────────────────────────────┤
│                    CORE (src/core/)                                       │
│                                                                          │
│  config.h                              types.h                           │
│  Pin mapping, I2C addr, PWM freq,      Direction enum (11 values)        │
│  encoder CPR, PID gains, timeouts,     State enum (5 values)             │
│  calibration params, geometry,          CmdType enum (9 values)          │
│  debug flags                            Command struct, MotorSpeeds      │
│                                         clamp(), absVal(), sign()        │
├──────────────────────────────────────────────────────────────────────────┤
│                    HARDWARE                                              │
│  Arduino Uno ↔ PCA9685 (I2C @ 400kHz) ↔ TB67H450 × 4 ↔ Motors/Encoders │
└──────────────────────────────────────────────────────────────────────────┘
```

### Why This Architecture?

- **Separation of concerns:** Changing the PCA9685 I2C address only touches `config.h`. Changing the PID gains only touches `config.h`. Swapping a motor's wiring only touches `config.h`. No "grep and replace across 10 files."
- **Testability:** The `env:hwtest` firmware uses only the HAL layer (motor + encoder + pca9685) without any application logic. You can test raw motor channels without PID interfering.
- **RAM efficiency:** On a 2KB RAM MCU, every byte matters. The code uses `uint8_t` where possible, `F()` macro for string literals (stored in Flash instead of RAM), and avoids the Arduino `String` class entirely (serial parser uses a fixed 32-byte char buffer).

---

## How a Command Travels from PC to Wheels

Here's exactly what happens when you send `FWD,100,1719` (go forward at speed 100 for ~10cm):

### Step 1: Serial Reception (`serial_cmd.cpp`)

Your PC sends the bytes `F`, `W`, `D`, `,`, `1`, `0`, `0`, `,`, `1`, `7`, `1`, `9`, `\n` over USB at 115200 baud.

`Serial_Cmd::update()` runs every main loop iteration. It reads one character at a time into a 32-byte buffer. When it sees `\n`, it null-terminates the buffer and calls `parse()`.

The parser uses a lookup table to match the command name:
```c
const MoveCmd moveCmds[] = {
    {"FWD",    Direction::FORWARD},
    {"BWD",    Direction::BACKWARD},
    {"LEFT",   Direction::LEFT},
    // ... 8 entries total
};
```

It matches "FWD", then reads `100` as speed and `1719` as ticks. Result: a `Command` struct with `type=MOVE, dir=FORWARD, speed=100, ticks=1719`.

### Step 2: State Machine Accepts Command (`robot.cpp`)

`handleCommand()` checks that `currentState == IDLE` (if we're already moving, it either preempts the old move or rejects with `BUSY`).

For MOVE commands, it:
1. Calls `Motion::setTarget(FORWARD, 100, 1719)`
2. Disables the watchdog timer (position moves have their own `MOVE_TIMEOUT_MS`)
3. Transitions to `State::MOVING`
4. Sends `OK` back to the PC

### Step 3: Motion Target Setup (`motion.cpp`)

`Motion::setTarget()` does several things:
1. **Resets all 4 encoders to zero** (this is the starting position)
2. **Resets all 4 PID controllers** (clears integral accumulators and previous tick counts)
3. **Computes the active motor bitmask:**
   - Calls `Mecanum::compute(FORWARD, 1, probe)` with a dummy speed to see which motors are non-zero
   - FORWARD uses all 4 motors → `activeMask = 0x0F`, `activeCount = 4`
   - (For DIAG_FL, only FR and RL are non-zero → `activeMask = 0x0A`, `activeCount = 2`)
4. **Initializes per-motor stall detection** (records current time as "last progress" for each motor)
5. **Starts motors at the minimum ramp speed** (40, not the full 100) to prevent wheel slip on startup

### Step 4: PID Runs 50 Times Per Second (`motion.cpp`)

Every 20ms, `Timer1` ISR fires and sets a flag. `main.cpp` loop detects the flag and calls `Robot::onTick()`, which calls `Motion::update()`.

Inside `Motion::update()` for position mode:

1. **Read all 4 encoders** as an atomic snapshot (interrupts disabled for ~16μs)
2. **Check completion:** have all active motors reached 1719 ticks? If yes, mark `target.active = false` and return.
3. **Check stall:** for each active motor, has it moved at least 50 ticks in the last 5 seconds? If not, report stall and stop.
4. **Compute speed profile:**
   - `avgPos` = average encoder position of active motors
   - `remain` = 1719 - avgPos
   - Apply acceleration ramp if avgPos < 300 ticks
   - Apply deceleration ramp if remain < 400 ticks
   - For short moves (≤ 250 ticks), enforce minimum speed of 80
5. **Compute kinematics:** `Mecanum::compute(FORWARD, adjustedSpeed, speeds)` → all 4 motors get the same speed (for FORWARD)
6. **Per-motor PID:** for each active motor:
   - Convert desired speed (0-255 PWM scale) to target tick rate: `targetRate = speed × calMaxTickrate[i] / 255`
   - Measure actual speed: `actualSpeed = |currentTicks - prevTicks|`
   - Calculate error: `error = |targetRate| - actualSpeed`
   - Feed-forward estimate: `FF = |targetRate| × 255 / calMaxTickrate[i]`
   - PI correction: `correction = 1.5 × error + 0.3 × integral`
   - Final PWM = FF + correction, clamped 0-255, with correct sign from kinematics
7. **Send to motors:** `Motor::setAll(pwm)` writes all 4 motors via I2C (2 batch transactions due to 32-byte Wire buffer limit)

### Step 5: Motor Output (`motor.cpp` → `pca9685.cpp`)

`Motor::setAll()` compares each motor's new speed with its current speed. If anything changed:

1. For each changed motor, it updates an 8-element PWM buffer (PCA9685 channels 8-15):
   - Speed +100 → IN1 gets duty cycle `100 × 16 = 1600` (12-bit PWM), IN2 gets 0
   - Speed -100 → IN1 gets 0, IN2 gets duty 1600
   - Speed 0 → both get 0 (coast)

2. `flushBuffer()` sends all 8 channels to the PCA9685 using `PCA9685::setMultiple()`. Since each channel needs 4 bytes (32 bytes total) but Arduino's Wire buffer is only 32 bytes (including the address byte), it splits into 2 batch transactions (6 + 2 channels). This is still much faster than 8 separate I2C writes.

3. The PCA9685 generates 1600Hz PWM signals. The TB67H450 H-bridges switch the motor power on/off at this rate, controlling motor speed.

### Step 6: Target Reached

When `activeAllReached(snap, 1719)` returns true (all 4 encoders ≥ 1719):

1. `Motion::update()` sets `target.active = false`
2. Back in `robot.cpp`, `handleMoving()` detects `Motion::isComplete() == true`
3. Calls `Motor::startBrake(40)` — all motors brake for 40ms (non-blocking)
4. Sends `DONE` to the PC
5. Transitions back to `State::IDLE`

The non-blocking brake timer is serviced by `Motor::updateBrake()` in the main loop. After 40ms, it automatically releases to coast.

---

## The Main Loop and Timing

### Entry Point (`main.cpp`)

```c
void setup() {
    Robot::init();   // Serial, motors, encoders, timer, load calibration
}

void loop() {
    if (Timer::consumeTick()) {   // Atomic check-and-clear (50Hz)
        Robot::onTick();           // PID + motor update
    }
    Robot::update();               // State machine + serial + telemetry
}
```

### Timer1 Architecture

Timer1 is configured in **CTC (Clear Timer on Compare) mode** at 50Hz:

```
16MHz clock ÷ 256 prescaler = 62,500 Hz counter
62,500 ÷ 50 Hz = 1250 counts → OCR1A = 1249
```

The ISR is intentionally minimal — it only sets a boolean flag:

```c
ISR(TIMER1_COMPA_vect) {
    if (tick) overrunCount++;   // Previous tick wasn't processed — bad!
    tick = true;                // ~1μs execution time
}
```

**Why not do PID work in the ISR?** Because the PID calls `Motor::setAll()` which does I2C communication. I2C uses `Wire.beginTransmission()` / `Wire.endTransmission()` which can take 200+ μs and may block. Doing that inside an ISR would delay other interrupts (including the encoder pin-change ISRs), causing missed encoder counts.

Instead, the main `loop()` uses `Timer::consumeTick()` which atomically checks and clears the flag (with interrupts disabled for ~3μs to prevent a race condition between checking and clearing).

### What Runs When

| When | What | Time Budget |
|------|------|-------------|
| Every loop iteration (~100μs) | `Robot::update()` — serial parsing, state machine, brake timer | < 100μs typical |
| Every 20ms (50Hz tick) | `Robot::onTick()` → `Motion::update()` — encoder read + PID + motor write | ~200-400μs |
| Every 50ms (20Hz) | Encoder telemetry print (during MOVING) or ODOM print (during VEL) | ~500μs |
| Every 1000ms (1Hz) | Encoder telemetry print (during IDLE) | ~500μs |
| Every 2000ms | I2C health check (ping PCA9685) | ~50μs |
| Asynchronous (any time) | Encoder pin-change ISRs (triggered by wheel rotation) | ~5μs each |

### Overrun Detection

If the main loop takes too long and misses a 20ms tick (e.g., a long serial print blocks for > 20ms), Timer1 ISR fires again before the previous tick was consumed. The `overrunCount` increments. This doesn't crash anything — the next tick just processes normally — but it means the PID ran at < 50Hz for that period.

---

## Robot State Machine

The robot is always in exactly one of 5 states:

```
                    ┌──────────────┐
        ┌───────────│     IDLE     │──────────┐
        │           │  (waiting)   │          │
        │           └──────┬───────┘          │
        │                  │                  │
     CALIB cmd        MOVE/VEL cmd      TMOTOR/TENC cmd
        │                  │                  │
        ▼                  ▼                  ▼
 ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
 │ CALIBRATING  │  │    MOVING    │  │   TESTING    │
 │ ~10 seconds  │  │ PID at 50Hz  │  │ Raw motor/   │
 │ 3 sessions   │  │ 20Hz telemetry│ │ encoder read │
 └──────┬───────┘  └──┬──────┬───┘  └──────┬───────┘
        │             │      │              │
     DONE/STOP    Target   Error          STOP
        │          reached  (stall/        │
        │             │      timeout/      │
        │             │      watchdog)     │
        └─────────────┴──────┴─────────────┘
                      │
                 Back to IDLE

 ┌──────────────┐
 │    ERROR     │ ← I2C fault (5 consecutive failures)
 │              │ → Auto-recovers to IDLE after 5 seconds
 └──────────────┘
```

### Boot Sequence (`Robot::init()`)

The exact initialization order matters — each step depends on the previous:

| Step | What | Serial Output | Details |
|------|------|---------------|---------|
| 1 | `Serial_Cmd::init()` | (Serial just became available) | Opens serial at 115200, waits for USB ready, 100ms stabilization |
| 2 | `Motor::init()` | `"Motor init..."` | I2C `Wire.begin()`, 3ms bus timeout, PCA9685 init (reset, set freq, all channels OFF) |
| 3 | `Encoder::init()` | `"Encoder init..."` | QGPMaker constructors already ran, reset all to zero, prints `ENC_RESET` |
| 4 | `Timer::init()` | `"Timer init..."` | Timer1 CTC mode, 50Hz ISR enabled |
| 5 | `Motion::init()` | `"Motion init..."` | Clear targets, reset PID state |
| 6 | EEPROM load | `"CALIB,loaded,FL:n,..."` or `"No calibration..."` | If marker 0xCB found: load per-motor maxTickrate (clamp <50 to default 150). Otherwise: use default 150 for all. |
| 7 | PS2 init (if enabled) | `"PS2 init..."` | Configure gamepad pins, detect controller |
| 8 | Set IDLE state | `"Init complete"`, `"READY"`, `"Robot initialized"` | State machine starts in IDLE, watchdog timer initialized |

### State Details

| State | Entry Condition | What Happens | Exit Condition | Output |
|-------|----------------|-------------|----------------|--------|
| **IDLE** | Power-on, or any operation finishes | Motors coast. 1Hz encoder telemetry. Checks for PS2 input. Accepts all commands. | Any movement/calibration/test command | `ENC,...` at 1Hz |
| **MOVING** | FWD/BWD/LEFT/etc./VEL command | PID runs at 50Hz. 20Hz telemetry (ENC in position mode, ODOM in VEL mode — selected by internal `velocityActive` flag). Checks stall/timeout/watchdog. Allows command preemption. | Target reached → `DONE`, stall → `ERROR: Stalled`, timeout → `ERROR: Timeout`, watchdog → `ERROR: Watchdog` | `ENC,...` or `ODOM,...` at 20Hz |
| **CALIBRATING** | CALIB command (only from IDLE) | Drives all motors at PWM=200 open-loop. 3 settle-measure-brake sessions. Reports per-motor tick rates. Saves to EEPROM. | Finishes → `DONE`, 30s timeout → `ERROR: Calib timeout`, STOP → `CALIB,aborted` | `CALIB,N/3,...` per session |
| **TESTING** | TMOTOR or TENC (from IDLE or TESTING) | TMOTOR: raw PWM to one motor (no PID), streams all 4 encoders. TENC: streams encoders (no motors, spin by hand). 20Hz output. | STOP only | `TEST,...` or `ENC,...` at 20Hz |
| **ERROR** | 5+ consecutive I2C failures | All motors coast. Most commands rejected. | Auto-recovers after 5 seconds | `ERROR: I2C fault` |

### Command Preemption

If the robot is MOVING and you send a new MOVE command, it doesn't reject with BUSY. Instead:
1. Coasts all motors immediately
2. Clears the old motion target
3. Sends `DONE` for the old command (so the PC isn't left waiting)
4. Starts the new command

This allows the PC to dynamically redirect the robot without a STOP-then-MOVE sequence.
