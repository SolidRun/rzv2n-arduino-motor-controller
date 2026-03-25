/**
 * @file mecanum.cpp
 * @brief Mecanum kinematics implementation
 *
 * Mecanum wheel force vectors:
 *   - Each wheel generates force at 45° to wheel axis
 *   - Combining 4 wheels allows omnidirectional movement
 *
 * Sign convention:
 *   - Positive speed = motor turns to move robot forward
 *   - Negative speed = motor turns to move robot backward
 */

#include "mecanum.h"
#include <Arduino.h>

namespace Mecanum {

void compute(Direction dir, int16_t speed, int16_t out[NUM_MOTORS]) {
    // Default: all stopped
    out[MOTOR_FL] = 0;
    out[MOTOR_FR] = 0;
    out[MOTOR_RL] = 0;
    out[MOTOR_RR] = 0;

    switch (dir) {
        case Direction::FORWARD:
            // All wheels forward
            out[MOTOR_FL] = speed;
            out[MOTOR_FR] = speed;
            out[MOTOR_RL] = speed;
            out[MOTOR_RR] = speed;
            break;

        case Direction::BACKWARD:
            // All wheels backward
            out[MOTOR_FL] = -speed;
            out[MOTOR_FR] = -speed;
            out[MOTOR_RL] = -speed;
            out[MOTOR_RR] = -speed;
            break;

        case Direction::LEFT:
            // Strafe left: FL/RR backward, FR/RL forward
            out[MOTOR_FL] = -speed;
            out[MOTOR_FR] = speed;
            out[MOTOR_RL] = speed;
            out[MOTOR_RR] = -speed;
            break;

        case Direction::RIGHT:
            // Strafe right: FL/RR forward, FR/RL backward
            out[MOTOR_FL] = speed;
            out[MOTOR_FR] = -speed;
            out[MOTOR_RL] = -speed;
            out[MOTOR_RR] = speed;
            break;

        case Direction::ROTATE_CCW:
            // Rotate left (counter-clockwise): left side back, right side forward
            out[MOTOR_FL] = -speed;
            out[MOTOR_FR] = speed;
            out[MOTOR_RL] = -speed;
            out[MOTOR_RR] = speed;
            break;

        case Direction::ROTATE_CW:
            // Rotate right (clockwise): left side forward, right side back
            out[MOTOR_FL] = speed;
            out[MOTOR_FR] = -speed;
            out[MOTOR_RL] = speed;
            out[MOTOR_RR] = -speed;
            break;

        case Direction::DIAG_FL:
            // Diagonal forward-left: only FR and RL
            out[MOTOR_FR] = speed;
            out[MOTOR_RL] = speed;
            break;

        case Direction::DIAG_FR:
            // Diagonal forward-right: only FL and RR
            out[MOTOR_FL] = speed;
            out[MOTOR_RR] = speed;
            break;

        case Direction::DIAG_BL:
            // Diagonal backward-left: only FR and RL backward
            out[MOTOR_FR] = -speed;
            out[MOTOR_RL] = -speed;
            break;

        case Direction::DIAG_BR:
            // Diagonal backward-right: only FL and RR backward
            out[MOTOR_FL] = -speed;
            out[MOTOR_RR] = -speed;
            break;

        case Direction::STOP:
        default:
            // All stopped (already zeroed)
            break;
    }
}

MotorSpeeds computeSpeeds(Direction dir, int16_t speed) {
    MotorSpeeds result;
    compute(dir, speed, result.m);
    return result;
}

int16_t calcAccelRamp(int16_t baseSpeed, int32_t currentPos) {
    if (currentPos >= ACCEL_RAMP_TICKS) {
        return baseSpeed;  // Past ramp zone, full speed
    }
    if (currentPos <= 0) {
        return ACCEL_MIN_SPEED;  // At start
    }
    // Linear ramp from ACCEL_MIN_SPEED to baseSpeed
    int16_t rampSpeed = ACCEL_MIN_SPEED +
        (int16_t)((int32_t)(baseSpeed - ACCEL_MIN_SPEED) * currentPos / ACCEL_RAMP_TICKS);
    return constrain(rampSpeed, (int16_t)ACCEL_MIN_SPEED, baseSpeed);
}

int16_t calcSlowdown(int16_t baseSpeed, int32_t remainingTicks) {
    if (remainingTicks >= SLOWDOWN_TICKS) {
        return baseSpeed;
    }

    if (remainingTicks <= 0) {
        return 0;
    }

    // Linear ramp down
    int16_t newSpeed = SLOWDOWN_MIN_SPEED +
                       (int16_t)(remainingTicks * SLOWDOWN_KP);

    if (newSpeed > baseSpeed) return baseSpeed;
    if (newSpeed < SLOWDOWN_MIN_SPEED) return SLOWDOWN_MIN_SPEED;

    return newSpeed;
}

void computeFromVelocity(int16_t vx_mm, int16_t vy_mm, int16_t wz_mrad,
                         int16_t out[NUM_MOTORS]) {
    // Mecanum inverse kinematics with real-world units (same as ODOM output).
    // Converts robot velocity (mm/s, mrad/s) → per-wheel tick-rate (ticks/period).

    // Angular velocity contribution at the wheel contact point (mm/s)
    float wz_mm = (float)MECANUM_LX_LY_MM * (float)wz_mrad / 1000.0f;

    // Per-wheel velocity in mm/s (standard mecanum IK)
    float wheel[NUM_MOTORS];
    wheel[MOTOR_FL] = (float)vx_mm - (float)vy_mm - wz_mm;
    wheel[MOTOR_FR] = (float)vx_mm + (float)vy_mm + wz_mm;
    wheel[MOTOR_RL] = (float)vx_mm + (float)vy_mm - wz_mm;
    wheel[MOTOR_RR] = (float)vx_mm - (float)vy_mm + wz_mm;

    // Convert mm/s → ticks per control period
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        out[i] = (int16_t)(wheel[i] * MM_S_TO_TICK_RATE);
    }
}

void forwardKinematics(const int16_t wheelDelta[NUM_MOTORS],
                       int16_t &vx_mm, int16_t &vy_mm, int16_t &wz_mrad) {
    // Convert ticks/period to mm/s per wheel
    // ODOM deltas span MOVING_TELEMETRY_MS (50ms), so rate = 1000/50 = 20Hz
    // mm_per_tick_per_s = π * WHEEL_DIAMETER_MM / ENCODER_CPR * (1000 / MOVING_TELEMETRY_MS)
    // = 3.14159 * 80 / 4320 * 20 ≈ 1.164 mm/s per tick/period
    const float K = 3.14159f * WHEEL_DIAMETER_MM * (1000.0f / MOVING_TELEMETRY_MS) / (float)ENCODER_CPR;

    float wFL = wheelDelta[MOTOR_FL] * K;
    float wFR = wheelDelta[MOTOR_FR] * K;
    float wRL = wheelDelta[MOTOR_RL] * K;
    float wRR = wheelDelta[MOTOR_RR] * K;

    // Forward kinematics (inverse of computeFromVelocity)
    vx_mm  = (int16_t)((wFL + wFR + wRL + wRR) * 0.25f);
    vy_mm  = (int16_t)((-wFL + wFR + wRL - wRR) * 0.25f);

    // Angular velocity: wz = sum / (4 * (lx + ly)), lx+ly = (wheelbase + track) / 2
    const float lxly = (WHEEL_BASE_MM + TRACK_WIDTH_MM) * 0.5f;
    float wz = (-wFL + wFR - wRL + wRR) / (4.0f * lxly);  // rad/s
    wz_mrad = (int16_t)(wz * 1000.0f);
}

} // namespace Mecanum
