/**
 * @file motion.cpp
 * @brief Motion control implementation
 *
 * Non-blocking motion control with encoder feedback.
 * Integrates mecanum kinematics with HAL layers.
 */

#include "motion.h"
#include "mecanum.h"
#include "../hal/motor.h"
#include "../hal/encoder.h"
#include <Arduino.h>

//==============================================================================
// Private State
//==============================================================================

namespace {
    MotionTarget target;
    bool velocityMode = false;  // true = PS2 control (no target)
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

    // Check if target reached
    if (Encoder::allReached(target.ticks)) {
        // Target reached - stop and deactivate
        target.active = false;
        return;  // Let state machine handle stopping
    }

    // Get encoder state
    Encoder::Snapshot snap;
    Encoder::getSnapshot(snap);

    // Calculate remaining distance
    int32_t avgPos = snap.average();
    int32_t remain = target.ticks - avgPos;

    if (remain <= 0) {
        target.active = false;
        return;
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

Direction getDirection() {
    return target.dir;
}

int32_t remaining() {
    if (!target.active || target.ticks <= 0) return 0;

    int32_t avg = Encoder::average();
    int32_t remain = target.ticks - avg;
    return (remain > 0) ? remain : 0;
}

void calibrate() {
    float gainSum[NUM_MOTORS] = {0, 0, 0, 0};

    for (int session = 0; session < CAL_SESSIONS; session++) {
        Encoder::resetAll();

        // Move forward at calibration speed
        int16_t speeds[NUM_MOTORS];
        Mecanum::compute(Direction::FORWARD, CAL_SPEED, speeds);
        Motor::setAll(speeds);

        // Wait for target (blocking)
        while (!Encoder::allReached(CAL_TICKS)) {
            delay(10);
        }

        Motor::brakeAll();
        delay(CAL_SETTLE_MS);

        // Read final positions
        Encoder::Snapshot snap;
        Encoder::getSnapshot(snap);

        // Calculate reference (average)
        float ref = (float)snap.average();

        // Accumulate gain corrections
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            int32_t absTicks = abs(snap.ticks[i]);
            if (absTicks > 0) {
                gainSum[i] += ref / (float)absTicks;
            } else {
                gainSum[i] += 1.0f;
            }
        }
    }

    // Calculate final gains
    float gains[NUM_MOTORS];
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        gains[i] = gainSum[i] / CAL_SESSIONS;
    }

    // Normalize (max = 1.0)
    float maxGain = gains[0];
    for (uint8_t i = 1; i < NUM_MOTORS; i++) {
        if (gains[i] > maxGain) maxGain = gains[i];
    }
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        gains[i] /= maxGain;
    }

    Motor::setAllGains(gains);
    delay(CAL_POST_MS);
    Motor::coastAll();
}

MotorSpeeds getSpeeds(Direction dir, int16_t speed) {
    return Mecanum::computeSpeeds(dir, speed);
}

} // namespace Motion
