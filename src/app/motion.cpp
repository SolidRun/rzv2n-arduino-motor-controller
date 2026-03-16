/**
 * @file motion.cpp
 * @brief Motion control implementation
 *
 * Non-blocking motion control with encoder feedback.
 * Integrates mecanum kinematics with HAL layers.
 */

#include "motion.h"
#include "mecanum.h"
#include "serial_cmd.h"
#include "../hal/motor.h"
#include "../hal/encoder.h"
#include <Arduino.h>
#include <EEPROM.h>

//==============================================================================
// Private State
//==============================================================================

namespace {
    // Per-motor max tick rate (from calibration or default)
    // Used in PID feed-forward: PWM = targetSpeed * 255 / calMaxTickrate[i]
    uint8_t calMaxTickrate[NUM_MOTORS] = {
        MAX_MOTOR_TICKRATE, MAX_MOTOR_TICKRATE,
        MAX_MOTOR_TICKRATE, MAX_MOTOR_TICKRATE
    };
    uint8_t calMaxTickrateRev[NUM_MOTORS] = {
        MAX_MOTOR_TICKRATE, MAX_MOTOR_TICKRATE,
        MAX_MOTOR_TICKRATE, MAX_MOTOR_TICKRATE
    };
    uint8_t calDeadZone[NUM_MOTORS] = { 0, 0, 0, 0 };

    // Calibration state machine
    enum class CalState : uint8_t {
        IDLE,
        // Dead-zone detection
        DZ_RAMP,        // Slowly increase PWM until motor moves
        DZ_BRAKE,       // Brake after dead-zone found for one motor
        // Forward speed measurement
        FWD_SETTLE, FWD_MEASURE, FWD_BRAKE,
        // Reverse speed measurement
        REV_SETTLE, REV_MEASURE, REV_BRAKE
    };
    CalState calState = CalState::IDLE;
    uint8_t calSession = 0;
    uint32_t calPhaseStart = 0;
    int32_t calStartTicks[NUM_MOTORS];
    int32_t calSessionTicks[NUM_MOTORS];  // Accumulated ticks across sessions

    // Dead-zone scan state
    uint8_t dzMotorIdx = 0;           // Which motor we're scanning
    uint8_t dzCurrentPWM = 0;         // Current ramp PWM
    int32_t dzPrevTicks[NUM_MOTORS];  // Encoder snapshot for dead-zone detection

    MotionTarget target;
    bool velocityMode = false;  // true = PS2 control (no target)

    // Active motor tracking for diagonal moves
    // Bitmask: bit 0=FL, 1=RL, 2=RR, 3=FR
    uint8_t activeMask = 0x0F;
    uint8_t activeCount = NUM_MOTORS;

    // Compute average encoder position of active motors only
    int32_t activeAverage(const Encoder::Snapshot& snap) {
        int32_t sum = 0;
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            if (activeMask & (1 << i)) {
                sum += abs(snap.ticks[i]);
            }
        }
        return (activeCount > 0) ? sum / activeCount : 0;
    }

    // Check if all active motors reached target
    bool activeAllReached(const Encoder::Snapshot& snap, int32_t tgt) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            if ((activeMask & (1 << i)) && abs(snap.ticks[i]) < tgt) {
                return false;
            }
        }
        return true;
    }

    // Per-motor stall detection
    int32_t lastProgressPos[NUM_MOTORS];   // Per-motor last position when progress made
    uint32_t lastProgressTime[NUM_MOTORS]; // Per-motor last progress timestamp
    bool stalled = false;
    uint8_t stalledMotor = 0;              // Which motor stalled (for reporting)

    //--- Per-motor velocity PID ---
    // Each motor has its own PI controller for closed-loop speed tracking.
    // Measures actual ticks/period from encoder, adjusts PWM to match target.
    struct MotorPID {
        int32_t prevTicks;     // Previous absolute encoder reading
        float integral;        // PI integral accumulator
    };

    MotorPID motorPID[NUM_MOTORS];
    int16_t velSetpoint[NUM_MOTORS];  // Velocity targets for VEL mode (ticks/period)

    // Compute PID output for one motor
    // PID controls SPEED (absolute value), kinematics controls DIRECTION (sign).
    // This is robust regardless of encoder sign convention.
    // Returns PWM value (-255 to 255)
    int16_t computePID(uint8_t i, int32_t currentTicks, int16_t targetRate) {
        // Measure actual speed (absolute ticks since last call)
        int16_t rawDelta = (int16_t)(currentTicks - motorPID[i].prevTicks);
        motorPID[i].prevTicks = currentTicks;
        int16_t actualSpeed = abs(rawDelta);  // Speed = magnitude only

        // If target is zero, reset and output nothing
        if (targetRate == 0) {
            motorPID[i].integral = 0.0f;
            return 0;
        }

        // PID works on absolute speed (magnitude)
        int16_t targetSpeed = abs(targetRate);
        int16_t error = targetSpeed - actualSpeed;

        // Integral with anti-windup
        motorPID[i].integral += (float)error;
        if (motorPID[i].integral > VEL_PID_IMAX) motorPID[i].integral = VEL_PID_IMAX;
        if (motorPID[i].integral < -VEL_PID_IMAX) motorPID[i].integral = -VEL_PID_IMAX;

        // Feed-forward: PWM estimate using direction-aware calibrated max tick rate
        uint8_t maxTR = (targetRate > 0) ? calMaxTickrate[i] : calMaxTickrateRev[i];
        float ff = (float)targetSpeed * 255.0f / (float)maxTR;

        // PI correction on top of feed-forward
        float correction = VEL_PID_KP * (float)error + VEL_PID_KI * motorPID[i].integral;

        int16_t pwm = (int16_t)(ff + correction);
        if (pwm > 255) pwm = 255;
        if (pwm < 0) pwm = 0;

        // Dead-zone enforcement: if PID wants to output something but below dead-zone, jump up
        if (pwm > 0 && pwm < calDeadZone[i]) pwm = calDeadZone[i];

        // Apply direction from kinematics (sign of targetRate)
        return (targetRate > 0) ? pwm : -pwm;
    }

    // Reset all PID state (call on new motion target)
    void resetPID() {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            motorPID[i].prevTicks = 0;
            motorPID[i].integral = 0.0f;
            velSetpoint[i] = 0;
        }
    }

}

//==============================================================================
// Public Interface
//==============================================================================

namespace Motion {

void init() {
    target.clear();
    velocityMode = false;
    resetPID();
}

void setTarget(Direction dir, int16_t speed, int32_t ticks) {
    Encoder::resetAll();
    resetPID();  // PID prevTicks = 0 (matches encoder reset)
    target.dir = dir;
    target.speed = constrain(speed, 0, SPEED_MAX);
    target.ticks = ticks;
    target.active = true;
    velocityMode = false;

    // Determine which motors are active for this direction
    int16_t probe[NUM_MOTORS];
    Mecanum::compute(dir, 1, probe);  // Dummy speed to get sign pattern
    activeMask = 0;
    activeCount = 0;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (probe[i] != 0) {
            activeMask |= (1 << i);
            activeCount++;
        }
    }

    // Reset per-motor stall detection
    uint32_t now = millis();
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        lastProgressPos[i] = 0;
        lastProgressTime[i] = now;
    }
    stalled = false;

    // Start motors at ramp starting speed (not full speed, prevents wheel slip)
    if (dir != Direction::STOP && speed > 0 && ticks > 0) {
        int16_t startSpeed = min(target.speed, (int16_t)ACCEL_MIN_SPEED);
        int16_t speeds[NUM_MOTORS];
        Mecanum::compute(dir, startSpeed, speeds);
        Motor::setAll(speeds);
    }
}

void setVelocity(Direction dir, int16_t speed) {
    if (dir == Direction::STOP || speed <= 0) {
        stop(false);
        return;
    }
    // Compute per-motor speeds from direction, then delegate to PID path
    int16_t speeds[NUM_MOTORS];
    Mecanum::compute(dir, constrain(speed, (int16_t)0, (int16_t)SPEED_MAX), speeds);
    setMotorVelocities(speeds);
}

void setMotorVelocities(const int16_t speeds[NUM_MOTORS]) {
    bool wasVelocityMode = velocityMode;

    target.active = true;
    target.ticks = 0;
    velocityMode = true;

    // Convert PWM-scale speeds to tick-rate targets
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        velSetpoint[i] = (int16_t)((int32_t)speeds[i] * calMaxTickrate[i] / 255);
    }

    // On first VEL command (mode transition), reset PID to avoid stale state
    if (!wasVelocityMode) {
        Encoder::Snapshot snap;
        Encoder::getSnapshot(snap);
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            motorPID[i].prevTicks = snap.ticks[i];
            motorPID[i].integral = 0.0f;
        }
    }
}

void stop(bool brake) {
    target.clear();
    velocityMode = false;

    if (brake) {
        Motor::brakeAll();
    } else {
        Motor::coastAll();
    }
}

void clearTarget() {
    target.active = false;
    target.ticks = 0;
}

void update() {
    // Skip if no active motion
    if (!target.active) return;

    // Get encoder state (single read for consistency)
    Encoder::Snapshot snap;
    Encoder::getSnapshot(snap);

    //--- Velocity mode: PID tracks velocity setpoints ---
    if (velocityMode) {
        int16_t pwm[NUM_MOTORS];
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            pwm[i] = computePID(i, snap.ticks[i], velSetpoint[i]);
        }
        Motor::setAll(pwm);
        return;
    }

    //--- Position mode: outer loop (position) + inner loop (PID velocity) ---
    if (target.ticks <= 0) return;

    // Check if all active motors reached target
    if (activeAllReached(snap, target.ticks)) {
        target.active = false;
        stalled = false;
        return;  // Let state machine handle stopping
    }

    // Calculate remaining distance (active motors only)
    int32_t avgPos = activeAverage(snap);
    int32_t remain = target.ticks - avgPos;

    if (remain <= 0) {
        target.active = false;
        stalled = false;
        return;
    }

    // Per-motor stall detection: check each active motor individually
    uint32_t now = millis();
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (!(activeMask & (1 << i))) continue;

        int32_t absPos = abs(snap.ticks[i]);
        if (absPos - lastProgressPos[i] >= STALL_MIN_PROGRESS) {
            lastProgressPos[i] = absPos;
            lastProgressTime[i] = now;
        } else if (now - lastProgressTime[i] > STALL_TIMEOUT_MS) {
            static const char* mnames[] = {"FL", "RL", "RR", "FR"};
            Serial.print(F("STALL,"));
            Serial.print(mnames[i]);
            Serial.print(F(",pos="));
            Serial.println(absPos);
            stalledMotor = i;
            stalled = true;
            target.active = false;
            Motor::coastAll();
            return;
        }
    }

    // Outer loop: compute desired speed from position profile
    int16_t adjustedSpeed = target.speed;
    adjustedSpeed = Mecanum::calcAccelRamp(adjustedSpeed, avgPos);
    adjustedSpeed = Mecanum::calcSlowdown(adjustedSpeed, remain);

    if (target.ticks <= SHORT_MOVE_TICKS) {
        adjustedSpeed = max(adjustedSpeed, (int16_t)MIN_WORK_SPEED);
    }

    // Compute target speed per motor from kinematics
    int16_t speeds[NUM_MOTORS];
    Mecanum::compute(target.dir, adjustedSpeed, speeds);

    // Inner loop: per-motor PID converts target speed to PWM
    int16_t pwm[NUM_MOTORS];
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (activeMask & (1 << i)) {
            // Convert PWM-scale speed to tick-rate target
            int16_t targetRate = (int16_t)((int32_t)speeds[i] * calMaxTickrate[i] / 255);
            pwm[i] = computePID(i, snap.ticks[i], targetRate);
        } else {
            // Inactive motor (diagonal move): keep PID tracking but output 0
            motorPID[i].prevTicks = snap.ticks[i];
            motorPID[i].integral = 0.0f;
            pwm[i] = 0;
        }
    }

    Motor::setAll(pwm);
}

bool isMoving() {
    return target.active;
}

bool isComplete() {
    if (!target.active) return true;
    if (velocityMode) return false;  // Velocity mode never "completes"
    if (target.ticks <= 0) return true;

    Encoder::Snapshot snap;
    Encoder::getSnapshot(snap);
    return activeAllReached(snap, target.ticks);
}

bool isStalled() {
    return stalled;
}

Direction getDirection() {
    return target.dir;
}

int32_t remaining() {
    if (!target.active || target.ticks <= 0) return 0;

    Encoder::Snapshot snap;
    Encoder::getSnapshot(snap);
    int32_t avg = activeAverage(snap);
    int32_t remain = target.ticks - avg;
    return (remain > 0) ? remain : 0;
}

MotorSpeeds getSpeeds(Direction dir, int16_t speed) {
    return Mecanum::computeSpeeds(dir, speed);
}

//==============================================================================
// Motor Calibration
//==============================================================================

// Helper: compute max tickrate from accumulated session ticks
static uint8_t computeMaxRate(int32_t totalTicks) {
    uint16_t totalPeriods = (uint16_t)CALIB_SESSIONS * (CALIB_MEASURE_MS / 20);
    uint16_t avgRate = (uint16_t)(totalTicks / totalPeriods);
    uint16_t maxRate = (uint16_t)((uint32_t)avgRate * 255 / CALIB_PWM);
    if (maxRate > 255) maxRate = 255;
    if (maxRate < 50) maxRate = 50;
    return (uint8_t)maxRate;
}

// Helper: start motors at given PWM (positive or negative for direction)
static void startMotorsAtPWM(int16_t pwmVal) {
    int16_t pwm[NUM_MOTORS] = { pwmVal, pwmVal, pwmVal, pwmVal };
    Motor::setAll(pwm);
}

void startCalibration() {
    calSession = 0;
    calPhaseStart = millis();
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        calSessionTicks[i] = 0;
        calDeadZone[i] = 0;
    }
    Encoder::resetAll();

    // Start with dead-zone detection: motor 0, PWM 0
    dzMotorIdx = 0;
    dzCurrentPWM = 0;

    // Snapshot encoder positions
    Encoder::Snapshot snap;
    Encoder::getSnapshot(snap);
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        dzPrevTicks[i] = snap.ticks[i];
    }

    // Start first motor at PWM 1
    dzCurrentPWM = 1;
    Motor::setDirect(dzMotorIdx, dzCurrentPWM);
    calState = CalState::DZ_RAMP;

    Serial.println(F("CALIB,start,phase:deadzone"));
}

CalResult updateCalibration() {
    if (calState == CalState::IDLE) return CalResult::DONE;

    static const char* mnames[] = {"FL", "RL", "RR", "FR"};
    uint32_t now = millis();
    uint32_t elapsed = now - calPhaseStart;

    //--- Dead-zone detection: ramp one motor at a time ---
    if (calState == CalState::DZ_RAMP) {
        if (elapsed >= CALIB_DZ_STEP_MS) {
            calPhaseStart = now;

            // Check if motor started moving
            Encoder::Snapshot snap;
            Encoder::getSnapshot(snap);
            int32_t delta = abs(snap.ticks[dzMotorIdx] - dzPrevTicks[dzMotorIdx]);
            dzPrevTicks[dzMotorIdx] = snap.ticks[dzMotorIdx];

            if (delta >= CALIB_DZ_THRESHOLD) {
                // Motor started moving — record dead-zone
                calDeadZone[dzMotorIdx] = dzCurrentPWM;
                Serial.print(F("CALIB,dz,"));
                Serial.print(mnames[dzMotorIdx]);
                Serial.print(':');
                Serial.println(dzCurrentPWM);

                // Stop this motor, move to next
                Motor::setDirect(dzMotorIdx, 0);
                Motor::brakeAll();
                calState = CalState::DZ_BRAKE;
                calPhaseStart = now;
            } else {
                // Increase PWM
                dzCurrentPWM++;
                if (dzCurrentPWM > CALIB_DZ_MAX_PWM) {
                    // Motor never moved — broken or very high friction
                    calDeadZone[dzMotorIdx] = CALIB_DZ_MAX_PWM;
                    Serial.print(F("CALIB,dz,"));
                    Serial.print(mnames[dzMotorIdx]);
                    Serial.print(F(":MAX!"));
                    Serial.println();

                    Motor::setDirect(dzMotorIdx, 0);
                    Motor::brakeAll();
                    calState = CalState::DZ_BRAKE;
                    calPhaseStart = now;
                } else {
                    Motor::setDirect(dzMotorIdx, dzCurrentPWM);
                }
            }
        }
    }
    else if (calState == CalState::DZ_BRAKE) {
        if (elapsed >= CALIB_BRAKE_MS) {
            dzMotorIdx++;
            if (dzMotorIdx >= NUM_MOTORS) {
                // Dead-zone done for all motors — start forward speed measurement
                Serial.println(F("CALIB,phase:forward"));
                calSession = 0;
                for (uint8_t i = 0; i < NUM_MOTORS; i++) calSessionTicks[i] = 0;
                Encoder::resetAll();
                startMotorsAtPWM(CALIB_PWM);
                calState = CalState::FWD_SETTLE;
                calPhaseStart = now;
            } else {
                // Next motor dead-zone scan
                Encoder::Snapshot snap;
                Encoder::getSnapshot(snap);
                for (uint8_t i = 0; i < NUM_MOTORS; i++) dzPrevTicks[i] = snap.ticks[i];
                dzCurrentPWM = 1;
                Motor::setDirect(dzMotorIdx, dzCurrentPWM);
                calState = CalState::DZ_RAMP;
                calPhaseStart = now;
            }
        }
    }

    //--- Forward speed measurement (3 sessions) ---
    else if (calState == CalState::FWD_SETTLE) {
        if (elapsed >= CALIB_SETTLE_MS) {
            Encoder::Snapshot snap;
            Encoder::getSnapshot(snap);
            for (uint8_t i = 0; i < NUM_MOTORS; i++) calStartTicks[i] = snap.ticks[i];
            calState = CalState::FWD_MEASURE;
            calPhaseStart = now;
        }
    }
    else if (calState == CalState::FWD_MEASURE) {
        if (elapsed >= CALIB_MEASURE_MS) {
            Encoder::Snapshot snap;
            Encoder::getSnapshot(snap);
            calSession++;

            Serial.print(F("CALIB,fwd,"));
            Serial.print(calSession);
            Serial.print('/');
            Serial.print(CALIB_SESSIONS);

            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                int32_t delta = abs(snap.ticks[i] - calStartTicks[i]);
                calSessionTicks[i] += delta;
                Serial.print(',');
                Serial.print(mnames[i]);
                Serial.print(':');
                Serial.print((int16_t)(delta / (CALIB_MEASURE_MS / 20)));
            }
            Serial.println();

            Motor::brakeAll();
            calState = CalState::FWD_BRAKE;
            calPhaseStart = now;
        }
    }
    else if (calState == CalState::FWD_BRAKE) {
        if (elapsed >= CALIB_BRAKE_MS) {
            if (calSession >= CALIB_SESSIONS) {
                // Forward done — compute and report
                Serial.print(F("CALIB,fwd,done"));
                for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                    calMaxTickrate[i] = computeMaxRate(calSessionTicks[i]);
                    Serial.print(',');
                    Serial.print(mnames[i]);
                    Serial.print(':');
                    Serial.print(calMaxTickrate[i]);
                }
                Serial.println();

                // Start reverse measurement
                Serial.println(F("CALIB,phase:reverse"));
                calSession = 0;
                for (uint8_t i = 0; i < NUM_MOTORS; i++) calSessionTicks[i] = 0;
                Encoder::resetAll();
                startMotorsAtPWM(-CALIB_PWM);
                calState = CalState::REV_SETTLE;
                calPhaseStart = now;
            } else {
                Encoder::resetAll();
                startMotorsAtPWM(CALIB_PWM);
                calState = CalState::FWD_SETTLE;
                calPhaseStart = now;
            }
        }
    }

    //--- Reverse speed measurement (3 sessions) ---
    else if (calState == CalState::REV_SETTLE) {
        if (elapsed >= CALIB_SETTLE_MS) {
            Encoder::Snapshot snap;
            Encoder::getSnapshot(snap);
            for (uint8_t i = 0; i < NUM_MOTORS; i++) calStartTicks[i] = snap.ticks[i];
            calState = CalState::REV_MEASURE;
            calPhaseStart = now;
        }
    }
    else if (calState == CalState::REV_MEASURE) {
        if (elapsed >= CALIB_MEASURE_MS) {
            Encoder::Snapshot snap;
            Encoder::getSnapshot(snap);
            calSession++;

            Serial.print(F("CALIB,rev,"));
            Serial.print(calSession);
            Serial.print('/');
            Serial.print(CALIB_SESSIONS);

            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                int32_t delta = abs(snap.ticks[i] - calStartTicks[i]);
                calSessionTicks[i] += delta;
                Serial.print(',');
                Serial.print(mnames[i]);
                Serial.print(':');
                Serial.print((int16_t)(delta / (CALIB_MEASURE_MS / 20)));
            }
            Serial.println();

            Motor::brakeAll();
            calState = CalState::REV_BRAKE;
            calPhaseStart = now;
        }
    }
    else if (calState == CalState::REV_BRAKE) {
        if (elapsed >= CALIB_BRAKE_MS) {
            if (calSession >= CALIB_SESSIONS) {
                // Reverse done — compute and report
                Serial.print(F("CALIB,rev,done"));
                for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                    calMaxTickrateRev[i] = computeMaxRate(calSessionTicks[i]);
                    Serial.print(',');
                    Serial.print(mnames[i]);
                    Serial.print(':');
                    Serial.print(calMaxTickrateRev[i]);
                }
                Serial.println();

                // All phases complete
                saveCalibToEEPROM();
                Serial.println(F("CALIB,saved"));
                calState = CalState::IDLE;
                return CalResult::DONE;
            } else {
                Encoder::resetAll();
                startMotorsAtPWM(-CALIB_PWM);
                calState = CalState::REV_SETTLE;
                calPhaseStart = now;
            }
        }
    }

    return CalResult::RUNNING;
}

void abortCalibration() {
    if (calState != CalState::IDLE) {
        Motor::brakeAll();
        calState = CalState::IDLE;
        Serial.println(F("CALIB,aborted"));
    }
}

bool isCalibrating() {
    return calState != CalState::IDLE;
}

const uint8_t* getCalMaxTickrate() {
    return calMaxTickrate;
}

const uint8_t* getCalMaxTickrateRev() {
    return calMaxTickrateRev;
}

const uint8_t* getCalDeadZone() {
    return calDeadZone;
}

bool loadCalibFromEEPROM() {
    uint8_t marker = EEPROM.read(EEPROM_CALIB_ADDR);

    if (marker == EEPROM_CALIB_MARKER) {
        // v2 format: forward + reverse + dead-zone
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            calMaxTickrate[i] = EEPROM.read(EEPROM_CALIB_ADDR + 1 + i);
            if (calMaxTickrate[i] < 50) calMaxTickrate[i] = MAX_MOTOR_TICKRATE;
        }
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            calMaxTickrateRev[i] = EEPROM.read(EEPROM_CALIB_ADDR + 5 + i);
            if (calMaxTickrateRev[i] < 50) calMaxTickrateRev[i] = MAX_MOTOR_TICKRATE;
        }
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            calDeadZone[i] = EEPROM.read(EEPROM_CALIB_ADDR + 9 + i);
        }
        return true;
    }
    else if (marker == EEPROM_CALIB_MARKER_V1) {
        // v1 format: forward only — load and use for both directions
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            calMaxTickrate[i] = EEPROM.read(EEPROM_CALIB_ADDR + 1 + i);
            if (calMaxTickrate[i] < 50) calMaxTickrate[i] = MAX_MOTOR_TICKRATE;
            calMaxTickrateRev[i] = calMaxTickrate[i];
        }
        return true;
    }

    return false;
}

void saveCalibToEEPROM() {
    EEPROM.update(EEPROM_CALIB_ADDR, EEPROM_CALIB_MARKER);
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        EEPROM.update(EEPROM_CALIB_ADDR + 1 + i, calMaxTickrate[i]);
    }
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        EEPROM.update(EEPROM_CALIB_ADDR + 5 + i, calMaxTickrateRev[i]);
    }
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        EEPROM.update(EEPROM_CALIB_ADDR + 9 + i, calDeadZone[i]);
    }
}

} // namespace Motion
