# Hardware Reference

## Block Diagram

```
YOUR PC                 ARDUINO UNO              PCA9685                  MOTORS
────────                ───────────              ───────                  ──────
                         ATmega328P
  USB Serial            16MHz, 2KB RAM
  115200 baud   ──────►  32KB Flash    ──I2C──►  PWM Driver   ──8 PWM──►  4x TB67H450
  (text commands)        1KB EEPROM    400kHz     16-ch, 12-bit           H-Bridge ICs
                                       SDA=A4     addr 0x60              (2 ch/motor)
                            ▲          SCL=A5     1600Hz                      │
                            │                                                 │
                       Pin-change                                        4x DC Motors
                       interrupts                                        (9600 RPM shaft,
                            │                                             90:1 gearbox)
                       4x Encoders ◄──── shaft coupling ─────────────────    │
                       4320 CPR                                          4x Mecanum
                       (QGPMaker)                                        Wheels (80mm)
```

## Components

| Part | What It Is | Why It's Needed | Key Specs |
|------|-----------|----------------|-----------|
| **Arduino Uno** | Microcontroller board | The brain — runs all code, reads sensors, controls motors | ATmega328P @ 16MHz, 2KB RAM, 32KB Flash, 1KB EEPROM |
| **PCA9685** | 16-channel PWM driver | The Arduino only has 6 PWM pins, but we need 8 (2 per motor x 4 motors). This chip gives us 16 PWM channels using just 2 I2C wires (SDA/SCL) | I2C addr 0x60, 12-bit resolution (0-4095), 1600Hz PWM frequency |
| **TB67H450** x4 | H-bridge motor driver | The Arduino can only output 5V at 40mA — not enough to spin a motor. The H-bridge switches the battery's high-current power using the Arduino's low-power signals | 2 inputs (IN1/IN2) per motor control direction and speed |
| **DC Motors** x4 | Geared DC motors | Actually spin the wheels | 9600 RPM shaft, 90:1 gearbox -> ~107 RPM at wheel -> ~154 ticks/period |
| **Encoders** x4 | Quadrature rotation sensors | Count how far each wheel has turned (like an odometer) | 4320 counts per revolution, pin-change interrupt driven |
| **Mecanum Wheels** x4 | Omnidirectional wheels | Angled rollers (45 degrees) let the robot strafe sideways | 80mm diameter, 251.3mm circumference |

## Motor Speed Derivation

The `MAX_MOTOR_TICKRATE = 150` default comes from the motor specs:

```
Motor shaft:  9600 RPM (manufacturer spec)
Gearbox:      90:1 reduction
Output RPM:   9600 / 90 = 106.7 RPM at the wheel
Revs/second:  106.7 / 60 = 1.778 rev/s
Ticks/second: 1.778 x 4320 CPR = 7680 ticks/s
Ticks/period: 7680 / 50 Hz = 153.6 ticks per 20ms control period

-> Rounded to 150 as conservative default (before calibration)
```

After calibration, each motor gets its own measured value (typically 148-162 range), stored in EEPROM.

## Robot Geometry

```
           ┌─────── 210mm (track width) ───────┐
           │                                    │
      FL ╲╲╲╲                              ╱╱╱╱ FR
           │                                    │       190mm
           │         ROBOT  BODY                │    (wheelbase)
           │                                    │
      RL ╱╱╱╱                              ╲╲╲╲ RR
           │                                    │
           └────────────────────────────────────┘

  Wheel diameter: 80mm
  Wheel circumference: pi x 80 = 251.3mm
  1 full wheel revolution = 4320 encoder ticks
```

---

## PCA9685 PWM Driver

### The I2C Bus

The Arduino communicates with the PCA9685 over I2C (2-wire serial):
- **SDA** = analog pin A4 (data)
- **SCL** = analog pin A5 (clock)
- **Speed** = 400kHz (I2C Fast Mode, set by build flag `-DTWI_FREQ=400000`)
- **Timeout** = 3ms (Wire.setWireTimeout) — if the bus hangs, it auto-resets

### Initialization Sequence

1. Software reset via I2C general call (addr 0x00, command 0x06)
2. Set MODE2: totem-pole outputs (stronger drive for H-bridge inputs)
3. Set MODE1: auto-increment enabled (allows writing multiple registers in one transaction)
4. Calculate prescaler for 1600Hz: `prescale = round((25MHz / 4096 / 1600 - 1) x 0.9) = 3`
5. Enter sleep mode -> write prescaler -> wake up -> enable restart + auto-increment
6. All 16 channels OFF initially

### Register Layout (Per Channel)

Each PCA9685 channel occupies 4 consecutive registers:

| Register | Name | Content |
|----------|------|---------|
| Base + 0 | ON_L | Low byte of "turn ON" phase (0-255) |
| Base + 1 | ON_H | High byte of "turn ON" phase (bit 4 = full ON when set) |
| Base + 2 | OFF_L | Low byte of "turn OFF" phase (0-255) |
| Base + 3 | OFF_H | High byte of "turn OFF" phase (bit 4 = full OFF when set) |

The base register for channel N is `0x06 + 4 x N`. For our motors (channels 8-15), the base starts at `0x06 + 32 = 0x26`.

**Special bit 12 (0x1000):** Setting bit 4 in the ON_H register (value 0x10) turns the channel fully ON regardless of the OFF register. Setting bit 4 in the OFF_H register turns it fully OFF. This is used for brake (both channels fully ON = 4095) and coast (both channels fully OFF = 0).

### PWM Batch Buffering

Instead of sending 8 separate I2C transactions (one per PCA9685 channel), the motor driver uses a **buffer-and-flush** strategy:

```
pwmBuffer[8]  <- channels 8 through 15 (all motor channels)

Motor::setAll():
  for each motor with changed speed:
    update pwmBuffer[IN1_index] and pwmBuffer[IN2_index]
  if anything changed:
    flushBuffer() -> PCA9685::setMultiple(startCh=8, count=8, pwmBuffer)
```

`PCA9685::setMultiple()` uses the PCA9685's auto-increment feature to write all 8 channels in one I2C transaction. Each channel needs 4 bytes (ON_L, ON_H, OFF_L, OFF_H), so 8 channels = 32 data bytes + 1 address byte = 33 bytes. Since Arduino's Wire buffer is only 32 bytes, it automatically splits into batches of 6 channels (1 addr + 24 data = 25 bytes each).

**Performance:** One batch I2C write takes ~200us at 400kHz. Eight individual writes would take ~1.6ms. This is critical because the PID runs at 50Hz (20ms period), and we need the motor update to be fast.

### Motor::set() vs Motor::setDirect()

| Function | Behavior | Used By |
|----------|----------|---------|
| `Motor::set(motor, speed)` | Only flushes I2C if speed actually changed (compares against cached value) | PID controller (via `Motor::setAll()`) |
| `Motor::setDirect(motor, speed)` | Always flushes I2C immediately, even if the speed is the same | `TMOTOR` diagnostic command |

`Motor::set()` is an optimization — during steady-state PID control, if the PID output hasn't changed, there's no reason to waste I2C bandwidth resending the same value.

`Motor::setDirect()` bypasses this because diagnostic tests need guaranteed output.

### PWM Speed Conversion

Motor speed (-255 to +255) is converted to 12-bit PCA9685 duty cycle (0 to 4095):

```
duty = |speed| x 16    (left shift by 4 bits)

Examples:
  speed = 100 -> duty = 1600/4095 = ~39% duty cycle
  speed = 200 -> duty = 3200/4095 = ~78% duty cycle
  speed = 255 -> duty = 4080/4095 = ~99.6% duty cycle
```

---

## TB67H450 H-Bridge Control

Each motor uses 2 PCA9685 channels (IN1 and IN2) to control the H-bridge:

| Desired Action | IN1 | IN2 | What Happens |
|---------------|-----|-----|-------------|
| **Forward** | PWM (speed) | 0V (off) | Motor spins forward, speed proportional to duty cycle |
| **Backward** | 0V (off) | PWM (speed) | Motor spins backward |
| **Brake** | HIGH (full on) | HIGH (full on) | Both sides of H-bridge ON — motor windings short-circuited, stops quickly |
| **Coast** | LOW (0V) | LOW (0V) | Both sides OFF — motor disconnected, freewheels with no current |

**Brake vs Coast:** Brake actively stops the motor (like pressing the brake pedal in a car). Coast just disconnects power (like putting a car in neutral). After reaching a target, the robot brakes for 40ms to stop quickly, then coasts to save power.

**Blocking vs Non-blocking brake:**
- `Motor::brakeAndRelease(ms)` — calls `brakeAll()`, then `delay(ms)`, then `coastAll()`. **Blocks the CPU** for the entire duration. Not used in normal operation.
- `Motor::startBrake(ms)` + `Motor::updateBrake()` — records the start time and returns immediately. `updateBrake()` is called every main loop iteration and checks if the brake duration has elapsed, then coasts. **Non-blocking** — this is what the firmware uses.
- `Motor::emergencyStop()` — equivalent to `brakeAll()` (immediate brake, no timed release).

---

## Motor Wiring Map

### Non-Sequential Index Mapping

> **Important:** Motor indices are NOT sequential (0,1,2,3 != FL,FR,RL,RR). The mapping is FL=0, RL=1, RR=2, FR=3. This matches the physical wiring. **Every array in the code** (motor channels, encoder objects, calibration data, stall timers) uses this ordering.

```
             ┌──── FRONT ────┐
             │                │
        FL (index 0)    FR (index 3)
        PCA CH 8,9      PCA CH 13,12
        Enc Port 1      Enc Port 4
        Enc Dir +1      Enc Dir -1

        RL (index 1)    RR (index 2)
        PCA CH 10,11    PCA CH 15,14
        Enc Port 2      Enc Port 3
        Enc Dir +1      Enc Dir -1
             │                │
             └──── REAR ─────┘
```

### Complete Wiring Table

| Motor | Index | PCA9685 IN1 | PCA9685 IN2 | Encoder Port | Encoder Dir | Notes |
|-------|-------|-------------|-------------|-------------|-------------|-------|
| **FL** (Front-Left) | 0 | CH 8 | CH 9 | Port 1 | +1 (normal) | Forward = positive ticks |
| **RL** (Rear-Left) | 1 | CH 10 | CH 11 | Port 2 | +1 (normal) | Forward = positive ticks |
| **RR** (Rear-Right) | 2 | CH 15 | CH 14 | Port 3 | **-1 (inverted)** | Right side spins opposite |
| **FR** (Front-Right) | 3 | CH 13 | CH 12 | Port 4 | **-1 (inverted)** | Right side spins opposite |

### Encoder Direction

When the robot moves forward, the left wheels spin clockwise but the right wheels spin counter-clockwise (they're mirrored). The `-1` multiplier on right-side encoders flips their count so that "forward" always produces positive ticks for all 4 wheels.

```c
// In encoder.cpp:
int32_t value = encoders[index].read();
return value * encDir[index];    // Multiply by +1 or -1
```

### How to Fix Wiring Mistakes

- **Wrong motor responds:** Change `*_IN1_CH` and `*_IN2_CH` values in `config.h`.
- **Motor spins wrong direction:** Swap its IN1 and IN2 channel numbers in `config.h`.
- **Encoder counts negative when forward:** Flip its `*_ENC_DIR` from `1` to `-1` in `config.h`.

---

## Encoders

### Specifications

| Property | Value |
|---------|-------|
| Type | Quadrature (2-channel, 4x decoding) |
| Counts per revolution (CPR) | 4,320 |
| Resolution | 0.058 mm per tick |
| Library | QGPMaker_Encoder (uses PinChangeInterrupt) |
| Interrupt type | Pin-change (all ports) |
| Sampling | 50 Hz atomic snapshot (all 4 read with interrupts disabled) |

### Distance Conversion Table

| Distance | Ticks | How to Calculate |
|----------|-------|-----------------|
| 1 mm | ~17 | `mm x 4320 / (pi x 80)` = `mm x 17.19` |
| 1 cm | ~172 | |
| 5 cm | ~860 | |
| 10 cm | ~1,719 | |
| 25.13 cm (1 revolution) | 4,320 | Exact (π × 80mm = 251.33mm) |
| 50 cm | ~8,594 | |
| 1 meter | ~17,189 | |

**Quick formula:** `ticks = distance_mm x 17.19`
**Reverse:** `distance_mm = ticks / 17.19` or `ticks x 0.058`

### Atomic Snapshot

On an 8-bit AVR (ATmega328P), reading a 32-bit integer takes 4 CPU instructions. An encoder ISR could fire between instruction 2 and 3, changing the value mid-read.

The solution: disable interrupts for the ~16us it takes to read all 4 encoders:

```c
void getSnapshot(Snapshot& out) {
    uint8_t sreg = SREG;        // Save interrupt state
    cli();                       // Disable all interrupts
    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
        out.ticks[i] = encoders[i].read() * encDir[i];
    }
    out.timestamp_us = micros();
    SREG = sreg;                 // Restore interrupt state
}
```

The `SREG` save/restore pattern (instead of `cli()`/`sei()`) preserves the previous interrupt state — important because this function might be called when interrupts are already disabled.

### Snapshot Helper Methods

| Method | Returns | Used For |
|--------|---------|----------|
| `average()` | Average of all 4 |ticks| | Position estimation |
| `minVal()` | Minimum |tick| across all 4 | (Available but not currently used) |
| `maxVal()` | Maximum |tick| across all 4 | (Available but not currently used) |
| `allReached(target)` | true if all 4 |ticks| >= target | Completion check (all-motors variant) |

The motion layer uses its own `activeAverage()` and `activeAllReached()` that respect the active motor bitmask for diagonal moves.
