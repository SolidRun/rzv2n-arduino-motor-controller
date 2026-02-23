# Motion Control

## Mecanum Wheel Kinematics

### How Mecanum Wheels Work

Each Mecanum wheel has small rollers mounted at 45 degrees to the wheel's axis. When the wheel spins, it produces a force that's 45 degrees offset from the wheel's rotation direction. By combining the forces from all 4 wheels, the robot can move in any direction.

```
         FRONT
    FL  ╲╲╲╲   ╱╱╱╱  FR        ╲╲ and ╱╱ show the direction of the rollers
   ┌────────────────────────┐
   │       ROBOT BODY       │
   └────────────────────────┘
    RL  ╱╱╱╱   ╲╲╲╲  RR
         REAR

 Key insight:
  - All wheels same direction      -> forward/backward
  - Left column vs right column    -> strafe left/right
  - Left side vs right side        -> rotate
  - Only 2 wheels (diagonal pair)  -> move diagonally
```

### Direction -> Motor Signs Table

Derived directly from `mecanum.cpp`. `+` = motor drives forward, `-` = backward, `0` = motor off (coast).

| Direction | FL (idx 0) | RL (idx 1) | RR (idx 2) | FR (idx 3) | Active Motors | Example Command |
|-----------|:---:|:---:|:---:|:---:|--------------|-----------------|
| **FORWARD** | + | + | + | + | All 4 | `FWD,100,1719` |
| **BACKWARD** | - | - | - | - | All 4 | `BWD,100,1719` |
| **LEFT** (strafe) | - | - | + | + | All 4 | `LEFT,80,1719` |
| **RIGHT** (strafe) | + | + | - | - | All 4 | `RIGHT,80,1719` |
| **ROTATE CCW** | - | + | - | + | All 4 | `TURN,60,500` |
| **ROTATE CW** | + | - | + | - | All 4 | `TURN,60,-500` |
| **DIAG FL** | 0 | + | 0 | + | RL + FR | `DIAGFL,80,1000` |
| **DIAG FR** | + | 0 | + | 0 | FL + RR | `DIAGFR,80,1000` |
| **DIAG BL** | 0 | - | 0 | - | RL + FR | `DIAGBL,80,1000` |
| **DIAG BR** | - | 0 | - | 0 | FL + RR | `DIAGBR,80,1000` |

### Active Motor Bitmask (Diagonal Moves)

Diagonal moves only use 2 of 4 motors. The firmware computes which motors are active:

```c
int16_t probe[NUM_MOTORS];
Mecanum::compute(dir, 1, probe);    // Dummy speed to get sign pattern
activeMask = 0;
activeCount = 0;
for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (probe[i] != 0) {
        activeMask |= (1 << i);     // Bit 0=FL, 1=RL, 2=RR, 3=FR
        activeCount++;
    }
}
```

For DIAG_FL: activeMask = 0b1010 = 0x0A (RL and FR), activeCount = 2.

**Inactive motor PID handling:** During diagonal moves, the 2 inactive motors still have their PID `prevTicks` updated to the current encoder reading and their integral accumulator zeroed. This prevents stale state from causing a PWM spike on the next move.

### Inverse Kinematics (VEL Command)

Standard mecanum formula to compute per-motor speeds from desired robot velocity:

```
FL = vx - vy - wz
FR = vx + vy + wz
RL = vx + vy - wz
RR = vx - vy + wz

Where:
  vx = forward velocity     (-255 to +255, positive = forward)
  vy = lateral velocity     (-255 to +255, positive = left strafe)
  wz = rotational velocity  (-255 to +255, positive = counter-clockwise)
```

If any computed motor speed exceeds 255, all 4 are scaled down proportionally to keep the ratio.

**Example:** `VEL,200,200,0` (forward + left strafe):
- FL = 0, FR = 400, RL = 400, RR = 0
- maxAbs = 400, scale = 255/400 = 0.6375
- Result: FL=0, FR=255, RL=255, RR=0 (diagonal forward-left at max speed)

### Forward Kinematics (Odometry)

In VEL mode, the firmware computes the robot's actual velocity from encoder readings and reports it as `ODOM,vx,vy,wz` at 20Hz.

```
K = pi x WHEEL_DIAMETER x ODOM_RATE / ENCODER_CPR
  = 3.14159 x 80mm x 20Hz / 4320
  = 1.164 mm/s per tick/period

ODOM_RATE = 1000 / MOVING_TELEMETRY_MS = 1000 / 50 = 20Hz

vx  = (FL + FR + RL + RR) / 4 x K        -> forward velocity (mm/s)
vy  = (-FL + FR + RL - RR) / 4 x K       -> lateral velocity (mm/s)
wz  = (-FL + FR - RL + RR) / (4 x L) x K -> angular velocity (rad/s)
      where L = (wheelbase + track_width) / 2 = (190 + 210) / 2 = 200mm
      Output converted to mrad/s (x 1000)
```

`FL`, `FR`, `RL`, `RR` here are encoder tick deltas since the last ODOM report (every 50ms, not absolute positions).

---

## PID Controller

### The Problem

If you set a motor to PWM=100, it might spin at 80 ticks/period under load, or 120 ticks/period on a smooth floor, or 60 ticks/period when the battery is low. Without feedback, you have no control over the actual speed.

The PID controller measures the actual wheel speed from the encoder and adjusts the PWM to match the desired speed. Think of it like cruise control in a car.

### Each Motor Has Its Own PID

There are 4 independent PI controllers, one per motor. Each tracks its own encoder and adjusts its own PWM. If one wheel hits a rough spot, only that wheel's PWM changes.

### Position Mode vs Velocity Mode

The PID controller is used in both modes, but the control architecture differs:

**Position mode (dual-loop):**
```
                    ┌──────────────┐     ┌─────────────┐     ┌──────────┐
Target ticks ──────►│  Outer loop  │────►│ Inner loop  │────►│  Motors  │
                    │  Position    │     │  PID per    │     │          │
Encoder ──────────►│  profiling   │     │  motor      │◄────│ Encoders │
feedback            │  (accel/     │     │  (FF + PI)  │     │          │
                    │   decel)     │     └─────────────┘     └──────────┘
                    └──────────────┘
 Outer loop: Reads avg encoder position -> computes accel/decel speed profile -> feeds speed to kinematics
 Inner loop: For each motor, converts speed to tick-rate target -> PID adjusts PWM to match
```

**Velocity mode (single-loop):**
```
                    ┌─────────────┐     ┌──────────┐
VEL command ──────►│  PID per    │────►│  Motors  │
(vx,vy,wz ->       │  motor      │     │          │
 per-motor         │  (FF + PI)  │◄────│ Encoders │
 velSetpoint)      └─────────────┘     └──────────┘

 No outer loop. VEL command directly sets velSetpoint for each motor. PID tracks it.
 Never "completes" — runs until STOP or watchdog.
```

The inner PI controller (feed-forward + proportional + integral) is identical in both modes.

### PID Step-by-Step (runs every 20ms, per motor)

```
┌─────────────────────────────────────────────────────────────────────┐
│ 1. READ ENCODER                                                     │
│    actualSpeed = |currentTicks - prevTicks|  (magnitude only)       │
│    prevTicks = currentTicks                                         │
│                                                                     │
│ 2. IF target is zero -> reset integral, output PWM=0, done         │
│                                                                     │
│ 3. COMPUTE ERROR                                                    │
│    error = |targetRate| - actualSpeed                               │
│                                                                     │
│ 4. ACCUMULATE INTEGRAL (with anti-windup)                           │
│    integral += error                                                │
│    integral = clamp(integral, -150, +150)                           │
│                                                                     │
│ 5. FEED-FORWARD (initial estimate based on calibration)             │
│    FF = |targetRate| x 255 / calMaxTickrate[motor]                  │
│                                                                     │
│ 6. PI CORRECTION                                                    │
│    correction = 1.5 x error + 0.3 x integral                       │
│                                                                     │
│ 7. FINAL OUTPUT                                                     │
│    pwm = FF + correction                                            │
│    pwm = clamp(pwm, 0, 255)                                        │
│    Apply sign from kinematics (+ or - for direction)                │
└─────────────────────────────────────────────────────────────────────┘
```

### Numerical Example

Motor FL, calibrated maxTickrate = 153 ticks/period. Target speed from kinematics = 100 (PWM scale).

**Tick 1:** Motor just started. actualSpeed = 0.
```
targetRate = 100 x 153 / 255 = 60 ticks/period
error = 60 - 0 = 60
integral = 60 (clamped to 150 max)
FF = 60 x 255 / 153 = 100
correction = 1.5 x 60 + 0.3 x 60 = 90 + 18 = 108
pwm = 100 + 108 = 208 -> motor gets strong initial kick
```

**Tick 5:** Motor is spinning, actualSpeed = 55.
```
error = 60 - 55 = 5
integral = 60 + 5 = 65 (accumulated)
FF = 100
correction = 1.5 x 5 + 0.3 x 65 = 7.5 + 19.5 = 27
pwm = 100 + 27 = 127 -> getting closer to target
```

**Tick 10:** Motor at steady state, actualSpeed = 60.
```
error = 60 - 60 = 0
integral = 65 + 0 = 65 (no change)
FF = 100
correction = 1.5 x 0 + 0.3 x 65 = 0 + 19.5 = 19.5
pwm = 100 + 19.5 ~ 120 -> stable output, integral provides offset
```

### Why Feed-Forward + PI (Not Just PID)?

- **Feed-forward** gives the motor a good initial estimate immediately — no waiting for the PID to "discover" the right PWM. This is why calibration matters.
- **P (proportional)** reacts instantly to speed errors.
- **I (integral)** slowly corrects persistent errors (e.g., friction).
- **No D (derivative)** — on a noisy encoder signal at 50Hz, the derivative term would amplify noise and cause jitter. Feed-forward already provides fast response.

### Anti-Windup (IMAX = 150)

If a motor stalls, the error stays large and the integral keeps growing. Clamping the integral at +/-150 prevents surge when the wheel is freed.

### PID Parameters

| Parameter | Value | Effect of Increasing | Effect of Decreasing |
|-----------|-------|---------------------|---------------------|
| **Kp** | 1.5 | Faster response, more oscillation | Slower response, smoother |
| **Ki** | 0.3 | Eliminates steady-state error faster, more overshoot | Slower correction, may have persistent error |
| **IMAX** | 150.0 | More integral authority, risk of overshoot after stall | Less steady-state correction range |
| **Control rate** | 50 Hz | Better tracking, more I2C traffic | Less responsive |

---

## Speed Ramping

| Phase | Condition | Speed Formula | Purpose |
|-------|-----------|---------------|---------|
| **Acceleration** | Position < 300 ticks (~1.7cm) | Linear ramp from 40 to `targetSpeed` | Prevents wheel slip |
| **Cruise** | 300 <= position <= (target - 400) | Full `targetSpeed` | Maximum speed |
| **Deceleration** | Remaining < 400 ticks (~2.3cm) | `60 + remaining x 0.05`, min 60 | Prevents overshoot |
| **Short moves** | Total ticks <= 250 (~1.5cm) | Minimum speed = 80 | Ensures motor starts |

**Example:** `FWD,150,5000` (move 5000 ticks at speed 150):
- Ticks 0-300: speed ramps 40 -> 150
- Ticks 300-4600: speed = 150 (full cruise)
- Ticks 4600-5000: speed ramps 150 -> 60 -> 0

---

## Motor Calibration

### Why Calibrate?

No two motors are identical. One might spin 10% faster than another at the same PWM. The PID uses a **feed-forward estimate**: "to achieve speed X, I probably need PWM Y." This estimate depends on each motor's real maximum speed.

| Scenario | Feed-Forward Accuracy | Result |
|----------|---------------------|--------|
| **No calibration** (default 150 for all) | ~+/-15% error | PID has to work harder, slower convergence |
| **After calibration** (e.g., FL:153, RL:150, RR:155, FR:159) | ~+/-2% error | PID converges in 1-2 ticks |

### Calibration Sequence

```
Send: CALIB
Response: CALIB,start

Phase 1: SETTLE (500ms)
  -> All 4 motors run at PWM=200 (open-loop, no PID)
  -> Wait 500ms for motors to reach steady state

Phase 2: MEASURE (2000ms)
  -> Record starting encoder positions
  -> Wait 2000ms
  -> Record ending encoder positions
  -> Per-motor ticks = |end - start|
  -> Report: CALIB,1/3,FL:120,RL:118,RR:122,FR:125

Phase 3: BRAKE (300ms)
  -> Brake all motors for 300ms
  -> If session < 3, go to Phase 1 (next session)

After 3 sessions:
  -> Average ticks/period across sessions
  -> Extrapolate to PWM=255:
    maxRate = avgRate x 255 / 200
  -> Clamp to 50-255 (sanity check)
  -> Report: CALIB,done,FL:153,RL:150,RR:155,FR:159
  -> Save to EEPROM
  -> Report: CALIB,saved
  -> Send: DONE
```

### The Extrapolation Math

```
totalPeriods = 3 sessions x (2000ms / 20ms) = 300 periods
avgRate = totalTicks / 300
maxRate = avgRate x 255 / 200

Example: Motor FL accumulated 36000 ticks across 3 sessions
  avgRate = 36000 / 300 = 120 ticks/period at PWM=200
  maxRate = 120 x 255 / 200 = 153 ticks/period at PWM=255
```

### EEPROM Storage Layout

| EEPROM Address | Content | Type |
|---------------|---------|------|
| 0 | Marker byte `0xCB` (validates data exists) | uint8_t |
| 1 | FL max tick rate | uint8_t |
| 2 | RL max tick rate | uint8_t |
| 3 | RR max tick rate | uint8_t |
| 4 | FR max tick rate | uint8_t |

Total: **5 bytes** out of 1024 bytes available. Uses `EEPROM.update()` (only writes if changed).

**Validation on save vs load:**
- **On save:** values clamped to 50-255.
- **On load:** values below 50 are replaced with the default 150 (not clamped to 50). This handles corrupted EEPROM data more safely.

### When to Re-Calibrate

- After first assembly
- After replacing a motor, wheel, or gearbox
- If the robot drifts sideways during straight-line moves
- After significant battery voltage change
- Calibration survives power-off (stored in EEPROM)
