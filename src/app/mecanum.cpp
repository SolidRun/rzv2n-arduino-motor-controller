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
            out[MOTOR_RL] = -speed;
            out[MOTOR_RR] = speed;
            break;

        case Direction::RIGHT:
            // Strafe right: FL/RR forward, FR/RL backward
            out[MOTOR_FL] = speed;
            out[MOTOR_FR] = -speed;
            out[MOTOR_RL] = speed;
            out[MOTOR_RR] = -speed;
            break;

        case Direction::ROTATE_CCW:
            // Rotate left (counter-clockwise): left side back, right side forward
            out[MOTOR_FL] = -speed;  
            out[MOTOR_FR] = speed;   
            out[MOTOR_RL] = speed;  
            out[MOTOR_RR] = -speed;   
            break;

        case Direction::ROTATE_CW:
            // Rotate right (clockwise): left side forward, right side back
            out[MOTOR_FL] = speed;
            out[MOTOR_FR] = -speed;
            out[MOTOR_RL] = -speed;
            out[MOTOR_RR] = speed;
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

void applySyncCorrection(int16_t speeds[NUM_MOTORS],
                        const int32_t encoderTicks[NUM_MOTORS],
                        float kp) {
    if (kp == 0.0f) return;

    // Calculate average absolute position
    int32_t sum = 0;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        sum += abs(encoderTicks[i]);
    }
    int32_t avg = sum / NUM_MOTORS;

    // Apply proportional correction
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        int32_t error = abs(encoderTicks[i]) - avg;
        int16_t correction = (int16_t)(error * kp);

        // Reduce speed if this motor is ahead
        if (speeds[i] > 0) {
            speeds[i] = constrain(speeds[i] - correction, 0, 255);
        } else if (speeds[i] < 0) {
            speeds[i] = constrain(speeds[i] + correction, -255, 0);
        }
    }
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

} // namespace Mecanum
