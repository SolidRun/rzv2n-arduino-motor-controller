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
| **LEFT** (strafe) | - | + | - | + | All 4 | `LEFT,80,1719` |
| **RIGHT** (strafe) | + | - | + | - | All 4 | `RIGHT,80,1719` |
| **ROTATE CCW** | - | - | + | + | All 4 | `TURN,60,500` |
| **ROTATE CW** | + | + | - | - | All 4 | `TURN,60,-500` |
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

### Why Motors Are Not Linear

DC motors do not have a linear relationship between PWM and speed. Several physical effects cause this:

```
Speed
  |                  .---*  ← back-EMF limits top speed
  |               ./
  |            ./
  |         ./        ← roughly linear in the middle
  |       /
  |     /
  |    |  ← sudden jump once static friction is overcome
  |    |
  |....|  ← dead zone: PWM applied but no movement
  +-------------------> PWM
     ^
     dead-zone end
```

1. **Static friction (stiction):** A motor needs extra force just to start moving. At low PWM, most of the energy fights friction instead of producing rotation. Once spinning, friction drops and speed jumps up.

2. **Back-EMF:** As a motor spins faster, it generates voltage opposing the supply voltage. The faster it goes, the more it resists. At high PWM, adding more power gives diminishing returns — the curve flattens.

3. **H-bridge voltage drop:** The TB67H450 driver loses ~0.5-1V internally. At low PWM (low effective voltage), this drop is a large percentage of the total. At high PWM, it's negligible. This makes the low end weaker than expected.

4. **Mechanical load:** Wheel weight, floor friction, and the robot's mass all resist motion, disproportionately affecting low speeds.

The **middle range** (roughly PWM 80-200) is approximately linear, which is why the current single-point calibration works well for most use cases. Inaccuracy appears mainly at very low or very high speeds.

### Current Calibration Limitations

The current calibration does one thing: measure how fast each motor spins at PWM=200, then scale that to PWM=255. This gives the PID a good starting estimate for "how much PWM do I need for this speed?" But it has three blind spots.

---

#### 1. Dead-Zone: The Motor Won't Move Below a Certain PWM

**The problem in simple terms:**

Every motor needs a minimum amount of power just to overcome friction and start spinning. Below that power, the motor gets electricity but the shaft doesn't move — it just hums.

Think of pushing a heavy box on the floor. If you push gently, nothing happens. You need to push hard enough to overcome friction first. Once the box starts sliding, it's easier to keep it moving.

```
What happens when PID outputs low PWM:

  PWM:     0   10   20   30   40   50   60   70
           |    |    |    |    |    |    |    |
  Motor A: -    -    -    -    GO!  →    →    →     (dead-zone = 40)
  Motor B: -    -    -    -    -    -    GO!  →     (dead-zone = 60)
  Motor C: -    -    -    GO!  →    →    →    →     (dead-zone = 30)
  Motor D: -    -    -    -    -    GO!  →    →     (dead-zone = 50)

  "-" = motor stuck (hums but no rotation)
  "GO!" = motor starts spinning
  "→" = motor spinning
```

**Why this matters for the robot:**

Imagine the robot needs to move slowly. The PID calculates that each motor needs PWM=45. Look at the table above:
- Motor A (dead-zone 40): spinning fine
- Motor B (dead-zone 60): **stuck** — not moving at all!
- Motor C (dead-zone 30): spinning fine
- Motor D (dead-zone 50): **stuck** — not moving at all!

Result: only 2 of 4 wheels are spinning. The robot moves, but crooked.

It gets worse: while Motor B is stuck, the PID sees "actual speed = 0, target speed = 45" and keeps increasing the integral term. After a few seconds, the integral has accumulated to a huge value. Now if the target speed increases to something above the dead-zone, the motor suddenly gets a massive PWM spike from the built-up integral, causing a jerk.

**How to fix it:**

During calibration, test each motor individually. Start at PWM=0, increase by 1 every 50ms, and watch the encoder. The first PWM value where the encoder moves = that motor's dead-zone. Store it (1 byte per motor = 4 bytes total in EEPROM).

Then in the PID: if the calculated PWM is between 1 and the dead-zone, jump it up to the dead-zone. If it's 0, keep it at 0 (motor should be stopped).

```
Before (without dead-zone knowledge):
  PID wants PWM 35 → outputs 35 → motor stuck → integral winds up → jerk later

After (with dead-zone knowledge):
  PID wants PWM 35 → dead-zone is 50 → outputs 50 → motor moves immediately → smooth
```

**Cost:** 4 bytes EEPROM, ~200 bytes Flash, ~5 seconds extra calibration time.

---

#### 2. Bidirectional Calibration: Forward is Not the Same as Backward

**The problem in simple terms:**

If you spin a motor forward, it might go at 150 ticks/period. Spin the same motor backward, it might only go 140 ticks/period. Why? Several reasons:
- The H-bridge driver (TB67H450) uses different transistors for forward vs backward, and they have slightly different resistance
- Mechanical friction in the gearbox may differ by direction (gear teeth mesh differently)
- Brush contact in the DC motor may favor one direction

The current calibration only measures forward. So the firmware thinks all motors do 150 ticks/period in both directions, but backward they actually do 140.

**Why this matters for strafing:**

When the robot strafes left, 2 motors go forward and 2 motors go backward:

```
Strafe LEFT:
  FL: backward (-speed)  ←  using forward calibration (WRONG by ~7%)
  FR: forward  (+speed)  ←  using forward calibration (correct)
  RL: forward  (+speed)  ←  using forward calibration (correct)
  RR: backward (-speed)  ←  using forward calibration (WRONG by ~7%)
```

The forward motors run at the correct speed, but the backward motors run ~7% slower than expected. The PID eventually corrects this, but during the first 100-200ms the forces don't cancel perfectly:

```
Expected (perfect strafe left):         Actual (uncalibrated backward):
       ← ← ← ←                               ← ← ← ←
  FL ↙         ↗ FR                       FL ↙(weak)    ↗ FR
       ROBOT                                    ROBOT
  RL ↖         ↘ RR                       RL ↖         ↘(weak) RR
       ← ← ← ←                               ↙ ← ← ↘

  Pure sideways                           Sideways + slight forward drift
```

The backward motors (FL, RR) produce slightly less force than expected, so the diagonal forces don't cancel the forward/backward component perfectly. The robot drifts at a slight angle instead of going straight sideways.

**How to fix it:**

After calibrating forward (3 sessions), run 3 more sessions in reverse. Store two values per motor: `calMaxTickrateFwd[]` and `calMaxTickrateRev[]`. The PID feed-forward picks the right one based on the motor's current direction.

**Cost:** 4 bytes EEPROM, ~100 bytes Flash, ~8 seconds extra calibration time.

---

#### 3. Multi-Point Speed Curve: The Motor is Not a Straight Line

**The problem in simple terms:**

The current calibration measures speed at ONE power level (PWM=200) and assumes everything else is proportional. Like measuring a car's fuel consumption at 100 km/h and assuming it's the same ratio at 20 km/h and 200 km/h — it's not.

Real example with numbers:

```
What we MEASURE during calibration:
  At PWM 200 → motor does 120 ticks/period

What we ASSUME (linear extrapolation):
  At PWM 100 → should do 60 ticks/period  (half the PWM = half the speed)
  At PWM 50  → should do 30 ticks/period  (quarter PWM = quarter speed)
  At PWM 250 → should do 150 ticks/period

What ACTUALLY happens:
  At PWM 100 → motor does 45 ticks/period  (25% less than assumed!)
  At PWM 50  → motor does 10 ticks/period  (67% less than assumed!)
  At PWM 250 → motor does 145 ticks/period (3% less — nearly flat at top)
```

```
Ticks/period
  |
150|                        .---* actual (flattens at top)
   |                    ./
120|              * ./         * assumed (straight line from origin)
   |            ./         /
 90|         ./         /
   |       /         /
 60|     /        / *          ← assumed 60, actual only 45
   |    |      /
 30|    |   / *                ← assumed 30, actual only 10
   |    | /
  0|....|/
   +--+--+--+--+--+--+---> PWM
      40 80 120 160 200 240
```

The gap between assumed and actual is biggest at low and high PWM. In the middle range (PWM 120-200) the assumption is close enough.

**Why this matters:**

The feed-forward estimate is wrong. At PWM 100, the PID thinks the motor should be doing 60 ticks/period, but it's only doing 45. So the PID sees a 15-tick error and keeps adding PWM to compensate. It works, but takes 3-5 extra control cycles (60-100ms) to settle. During that time, the motor speed oscillates.

For most use cases this is fine — the PID handles it. The effect is only noticeable at very low speeds where the error percentage is large.

**How to fix it:**

Measure at 5 PWM levels: 80, 120, 160, 200, 240. Store all 5 measurements per motor (5 bytes × 4 motors = 20 bytes EEPROM). When the PID needs a feed-forward estimate at, say, PWM 150, it interpolates between the PWM=120 and PWM=160 measurements instead of extrapolating from a single point.

```
Before (single-point, linear assumption):
  Target: 45 ticks → FF = 45 × 255 / 150 = 76.5 PWM
  Actual speed at PWM 76: only 30 ticks (error = 15 ticks = 33%!)

After (multi-point, interpolated):
  Target: 45 ticks → lookup: 45 ticks happens at ~PWM 100 → FF = 100
  Actual speed at PWM 100: 45 ticks (error ≈ 0!)
```

**Cost:** 20 bytes EEPROM, ~40 bytes RAM, ~500 bytes Flash, ~40 seconds calibration time. This is the most expensive improvement.

---

### What's Realistic on ATmega328P

The ATmega328P is a small microcontroller. Think of it like a tiny house — you can't put everything in it, so you need to choose what matters most.

**How much space do we have?**

```
                    Total     Used      Free
  RAM (variables):  2048 B    1103 B    945 B    ← this is the tight one
  Flash (code):     32256 B   18854 B   13402 B  ← comfortable
  EEPROM (saved):   1024 B    13 B      1011 B   ← plenty
```

RAM is the bottleneck. Every new variable, array, or buffer eats into the 945 free bytes. If RAM runs out, the Arduino crashes randomly (stack overflow).

**What is implemented and what it cost:**

| Improvement | EEPROM | RAM | Flash | Calibration Time | Status |
|-------------|--------|-----|-------|-----------------|--------|
| v1: Forward-only calibration | 5 B | 0 | baseline | ~8s | Replaced by v2 |
| v2: + Dead-zone detection | +4 B | +18 B | +1758 B | +24s | **Implemented** |
| v2: + Bidirectional (fwd+rev) | +4 B | (included above) | (included above) | +8s | **Implemented** |
| Not implemented: Multi-point curve | +20 B | +40 B | +500 B | +40s | Not needed |

**Actual build with v2 calibration:** RAM 53.9% (1103/2048), Flash 58.5% (18854/32256). Plenty of headroom.

**Why multi-point curve was not implemented:**

The PID already handles feed-forward inaccuracy — that's its job. A wrong feed-forward just means the PID takes 2-3 extra ticks (40-60ms) to converge. You won't notice 60ms. This is only worth it if you need instant, perfect speed response (e.g., high-speed path following). For normal movement commands, the PID is fast enough.

### Calibration Sequence (v2)

The `CALIB` command now runs three phases automatically:

```
Send: CALIB
Response: CALIB,start,phase:deadzone

=== Phase 1: Dead-Zone Detection (~24s max) ===
For each motor (FL, RL, RR, FR), one at a time:
  -> Start motor at PWM=1
  -> Every 50ms, increase PWM by 1
  -> Check encoder: if >= 3 ticks moved, that's the dead-zone
  -> Report: CALIB,dz,FL:45
  -> Brake 300ms, then next motor
  -> If PWM reaches 120 with no movement: CALIB,dz,FL:MAX!

=== Phase 2: Forward Speed Measurement (~8.4s) ===
Response: CALIB,phase:forward

3 sessions, each:
  -> SETTLE: All motors at PWM=200 forward, wait 500ms
  -> MEASURE: Record encoder ticks over 2000ms
  -> Report: CALIB,fwd,1/3,FL:120,RL:118,RR:122,FR:125
  -> BRAKE: 300ms

After 3 sessions:
  -> Extrapolate to PWM=255: maxRate = avgRate x 255 / 200
  -> Report: CALIB,fwd,done,FL:153,RL:150,RR:155,FR:159

=== Phase 3: Reverse Speed Measurement (~8.4s) ===
Response: CALIB,phase:reverse

Same as forward but motors run at PWM=-200 (backward).
  -> Report: CALIB,rev,1/3,FL:118,RL:116,RR:120,FR:123
  -> Report: CALIB,rev,done,FL:150,RL:148,RR:153,FR:157

=== Completion ===
  -> Save all data to EEPROM (v2 format)
  -> Report: CALIB,saved
  -> Send: DONE

Total time: ~40 seconds (dead-zone varies by motor friction)
Safety timeout: 60 seconds
```

### How the PID Uses Calibration Data

**Direction-aware feed-forward:**
```
if motor is going forward:
    FF = targetSpeed x 255 / calMaxTickrate[motor]       <- forward calibration
else:
    FF = targetSpeed x 255 / calMaxTickrateRev[motor]     <- reverse calibration
```

**Dead-zone enforcement:**
```
if PID output > 0 and PID output < calDeadZone[motor]:
    PID output = calDeadZone[motor]    <- jump past the dead-zone
```

This means the motor always gets enough PWM to actually move. No more wasted time in the dead-zone while the integral winds up.

**Known limitation:** Dead-zone is only measured in the forward direction. A motor's reverse dead-zone may differ slightly (different H-bridge transistor, different friction). In practice the difference is small (1-3 PWM) and the PID compensates within one tick. If this becomes a problem, the dead-zone scan could be extended to measure both directions.

### The Extrapolation Math

```
totalPeriods = 3 sessions x (2000ms / 20ms) = 300 periods
avgRate = totalTicks / 300
maxRate = avgRate x 255 / 200

Example: Motor FL accumulated 36000 ticks across 3 sessions
  avgRate = 36000 / 300 = 120 ticks/period at PWM=200
  maxRate = 120 x 255 / 200 = 153 ticks/period at PWM=255
```

### EEPROM Storage Layout (v2)

| EEPROM Address | Content | Type |
|---------------|---------|------|
| 0 | Marker byte `0xCC` (v2) or `0xCB` (v1, legacy) | uint8_t |
| 1-4 | Forward max tick rate (FL, RL, RR, FR) | uint8_t x4 |
| 5-8 | Reverse max tick rate (FL, RL, RR, FR) | uint8_t x4 |
| 9-12 | Dead-zone PWM (FL, RL, RR, FR) | uint8_t x4 |

Total: **13 bytes** out of 1024 bytes available. Uses `EEPROM.update()` (only writes if changed).

**Backward compatibility:** If the EEPROM contains a v1 marker (`0xCB`), the firmware loads forward tickrate values and copies them to both forward and reverse arrays. Dead-zones default to 0 (disabled).

**Validation on load:**
- Tick rate values below 50 are replaced with the default `MAX_MOTOR_TICKRATE` (146)
- Dead-zone values are used as-is (0 = no dead-zone enforcement)

### Smart Calibration GUI

The Python GUI (`tools/robot_test.py`) has 4 tabs: **Position**, **Velocity**, **Motor Test**, and **Smart Calibration**. Live telemetry (encoder ticks + odometry) is always visible above the tabs.

The **Smart Calibration** tab contains:

1. **Motor Calibration** — runs the full `CALIB` command and displays results in a dashboard:
   - Dead-zone value per motor (minimum PWM to move)
   - Forward and reverse max tickrate per motor
   - Forward/reverse asymmetry (absolute difference + percentage, color-coded)
   - Legend explaining each column

2. **Ticks-per-mm Measurement** — tests if ticks-to-distance is consistent:
   - Drive in any direction (FWD/BWD/LEFT/RIGHT) at any speed for any ticks
   - 6 quick-test presets (FWD slow/med/fast, BWD, LEFT, RIGHT)
   - Enter measured distance → calculates ticks/mm vs theoretical (17.19)
   - Results table with all measurements
   - Auto-analysis: per-direction averages, strafe roller slip %, speed dependency, verdict

3. **Export & Update** — saves calibration data:
   - **Update config.h**: writes average forward tickrate as new `MAX_MOTOR_TICKRATE` default
   - **Export JSON**: saves all data (dead-zone, tickrates, asymmetry, ticks/mm measurements) to `tools/calibration.json`

### When to Re-Calibrate

- After first assembly
- After replacing a motor, wheel, or gearbox
- If the robot drifts sideways during straight-line moves
- After significant battery voltage change
- Calibration survives power-off (stored in EEPROM)
