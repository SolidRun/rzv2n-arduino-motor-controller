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

//==============================================================================
// Private State
//==============================================================================

namespace {
    MotionTarget target;
    bool velocityMode = false;  // true = PS2 control (no target)
    volatile bool calibrationAbort = false;  // Flag to abort calibration
    volatile bool calibrationRunning = false;

    // Stall detection
    int32_t lastProgressPos = 0;      // Last encoder position when progress was made
    uint32_t lastProgressTime = 0;    // Time when last progress was detected
    bool stalled = false;             // Flag indicating motion has stalled

    // Check serial for STOP command during calibration
    void checkForStopCommand() {
        Serial_Cmd::update();
        if (Serial_Cmd::hasCommand()) {
            Command cmd;
            if (Serial_Cmd::getCommand(cmd)) {
                if (cmd.type == CmdType::STOP) {
                    calibrationAbort = true;
                    Serial_Cmd::sendMessage("Calibration aborted");
                } else {
                    // Other commands rejected during calibration
                    Serial_Cmd::sendBusy();
                }
            }
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
}

void setTarget(Direction dir, int16_t speed, int32_t ticks) {
    // Reset encoders for fresh target
    Encoder::resetAll();

    target.dir = dir;
    target.speed = constrain(speed, 0, SPEED_MAX);
    target.ticks = ticks;
    target.active = true;
    velocityMode = false;

    // Reset stall detection
    lastProgressPos = 0;
    lastProgressTime = millis();
    stalled = false;

    // Start motors immediately
    if (dir != Direction::STOP && speed > 0 && ticks > 0) {
        int16_t speeds[NUM_MOTORS];
        Mecanum::compute(dir, target.speed, speeds);
        Motor::setAll(speeds);
    }
}

void setVelocity(Direction dir, int16_t speed) {
    target.dir = dir;
    target.speed = constrain(speed, 0, SPEED_MAX);
    target.ticks = 0;
    target.active = (dir != Direction::STOP);
    velocityMode = true;

    // Apply immediately
    if (dir != Direction::STOP && speed > 0) {
        int16_t speeds[NUM_MOTORS];
        Mecanum::compute(dir, target.speed, speeds);
        Motor::setAll(speeds);
    } else {
        Motor::coastAll();
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

    // Velocity mode - nothing to track
    if (velocityMode) return;

    // No target ticks = nothing to check
    if (target.ticks <= 0) return;

    // Get encoder state (single read for consistency)
    Encoder::Snapshot snap;
    Encoder::getSnapshot(snap);

    // Check if target reached using the snapshot we just took
    if (snap.allReached(target.ticks)) {
        // Target reached - stop and deactivate
        target.active = false;
        stalled = false;
        return;  // Let state machine handle stopping
    }

    // Calculate remaining distance using same snapshot
    int32_t avgPos = snap.average();
    int32_t remain = target.ticks - avgPos;

    if (remain <= 0) {
        target.active = false;
        stalled = false;
        return;
    }

    // Stall detection: check if we're making progress
    int32_t progressMade = avgPos - lastProgressPos;
    if (progressMade >= STALL_MIN_PROGRESS) {
        // We made progress, reset stall timer
        lastProgressPos = avgPos;
        lastProgressTime = millis();
    } else {
        uint32_t elapsed = millis() - lastProgressTime;
        if (elapsed > STALL_TIMEOUT_MS) {
            // No progress for too long - stalled!
            Serial.print(F("STALL! pos="));
            Serial.print(avgPos);
            Serial.print(F(" elapsed="));
            Serial.println(elapsed);
            stalled = true;
            target.active = false;
            Motor::coastAll();
            return;  // Let state machine detect via isComplete() or isStalled()
        }
    }

    // Calculate adjusted speed with slowdown
    int16_t adjustedSpeed = Mecanum::calcSlowdown(target.speed, remain);

    // Compute motor speeds
    int16_t speeds[NUM_MOTORS];
    Mecanum::compute(target.dir, adjustedSpeed, speeds);

    // Apply motor synchronization
    Mecanum::applySyncCorrection(speeds, snap.ticks, SYNC_KP);

    // Update motors
    Motor::setAll(speeds);
}

bool isMoving() {
    return target.active;
}

bool isComplete() {
    if (!target.active) return true;
    if (velocityMode) return false;  // Velocity mode never "completes"
    if (target.ticks <= 0) return true;

    return Encoder::allReached(target.ticks);
}

bool isStalled() {
    return stalled;
}

Direction getDirection() {
    return target.dir;
}

int32_t remaining() {
    if (!target.active || target.ticks <= 0) return 0;

    int32_t avg = Encoder::average();
    int32_t remain = target.ticks - avg;
    return (remain > 0) ? remain : 0;
}

/**
 * @brief Run a single calibration session for one direction
 * @param dir Direction to test
 * @param gainSum Array to accumulate gain corrections
 * @param sampleCount Array to track samples per motor
 * @return 1 if session was valid, 0 if invalid, -1 if aborted
 */
namespace {
    int8_t calibrateDirection(Direction dir, float gainSum[NUM_MOTORS],
                              uint8_t sampleCount[NUM_MOTORS]) {
        if (calibrationAbort) return -1;

        Encoder::resetAll();

        // Compute expected motor activity for this direction
        int16_t speeds[NUM_MOTORS];
        Mecanum::compute(dir, CAL_SPEED, speeds);
        Motor::setAll(speeds);

        // Wait for target with timeout protection and abort check
        uint32_t startTime = millis();
        bool timedOut = false;
        while (!Encoder::allReached(CAL_TICKS)) {
            // Check for STOP command from serial
            checkForStopCommand();
            // Check for abort request
            if (calibrationAbort) {
                Motor::coastAll();
                return -1;  // Aborted
            }
            if (millis() - startTime > CAL_TIMEOUT_MS) {
                timedOut = true;
                break;
            }
            delay(10);
        }

        Motor::brakeAll();

        if (timedOut) {
            Motor::coastAll();
            return 0;  // Invalid but not aborted
        }

        // Settle delay with abort check
        uint32_t settleStart = millis();
        while (millis() - settleStart < CAL_SETTLE_MS) {
            checkForStopCommand();
            if (calibrationAbort) {
                Motor::coastAll();
                return -1;
            }
            delay(10);
        }

        // Read final positions
        Encoder::Snapshot snap;
        Encoder::getSnapshot(snap);

        // Calculate reference (average of active motors only)
        int32_t sum = 0;
        uint8_t activeCount = 0;
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            if (speeds[i] != 0) {  // Only count motors that were active
                sum += abs(snap.ticks[i]);
                activeCount++;
            }
        }

        if (activeCount == 0) return 0;
        float ref = (float)sum / activeCount;

        // Accumulate gain corrections only for active motors
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            if (speeds[i] != 0) {  // Only calibrate motors that were active
                int32_t absTicks = abs(snap.ticks[i]);
                if (absTicks >= 100) {  // Minimum valid movement
                    gainSum[i] += ref / (float)absTicks;
                    sampleCount[i]++;
                }
            }
        }

        return 1;  // Valid session
    }
}

void abortCalibration() {
    calibrationAbort = true;
}

bool isCalibrating() {
    return calibrationRunning;
}

bool calibrate() {
    // Forward-only calibration:
    // All 4 motors active, measures relative performance
    // and adjusts gains to equalize wheel speeds

    calibrationRunning = true;
    calibrationAbort = false;

    float gainSum[NUM_MOTORS] = {0, 0, 0, 0};
    uint8_t sampleCount[NUM_MOTORS] = {0, 0, 0, 0};
    bool aborted = false;

    // Run multiple calibration sessions in forward direction
    for (uint8_t session = 0; session < CAL_SESSIONS && !aborted; session++) {
        int8_t result = calibrateDirection(Direction::FORWARD, gainSum, sampleCount);
        if (result == -1) {
            aborted = true;
            break;
        }

        // Brief pause between sessions with abort check
        if (!aborted && session < CAL_SESSIONS - 1) {
            uint32_t pauseStart = millis();
            while (millis() - pauseStart < CAL_SETTLE_MS) {
                checkForStopCommand();
                if (calibrationAbort) {
                    aborted = true;
                    break;
                }
                delay(10);
            }
        }
    }

    calibrationRunning = false;

    if (aborted) {
        Motor::coastAll();
        return false;  // Calibration was aborted
    }

    // Check that each motor got enough samples
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (sampleCount[i] < 2) {
            Motor::coastAll();
            return false;  // Not enough valid samples for this motor
        }
    }

    // Calculate final gains (average per motor)
    float gains[NUM_MOTORS];
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        gains[i] = gainSum[i] / sampleCount[i];
    }

    // Normalize (max = 1.0) with protection against zero
    float maxGain = 0.0f;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (gains[i] > maxGain) maxGain = gains[i];
    }
    if (maxGain <= 0.0f) {
        Motor::coastAll();
        return false;  // All gains are zero - calibration failed
    }
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        gains[i] /= maxGain;
    }

    Motor::setAllGains(gains);
    delay(CAL_POST_MS);
    Motor::coastAll();
    return true;  // Calibration successful
}

MotorSpeeds getSpeeds(Direction dir, int16_t speed) {
    return Mecanum::computeSpeeds(dir, speed);
}

} // namespace Motion
