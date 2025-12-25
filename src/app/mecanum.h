/**
 * @file mecanum.h
 * @brief Mecanum drive kinematics
 *
 * Calculates motor speeds for 4-wheel mecanum omnidirectional drive.
 *
 * Wheel layout (top view):
 *   FL \\  // FR     Rollers at 45° angles
 *   RL //  \\ RR
 *
 * This configuration allows:
 *   - Forward/Backward: All wheels same direction
 *   - Strafe Left/Right: Diagonal pairs opposite
 *   - Rotation: Left vs Right sides opposite
 *   - Diagonal: Only two wheels active
 */

#ifndef APP_MECANUM_H
#define APP_MECANUM_H

#include <stdint.h>
#include "../core/config.h"
#include "../core/types.h"

namespace Mecanum {

/**
 * @brief Compute motor speeds for a direction
 * @param dir Movement direction
 * @param speed Base speed (0-255)
 * @param out Output array of 4 motor speeds (FL, FR, RL, RR)
 *
 * Output speeds are signed: positive = forward, negative = backward
 */
void compute(Direction dir, int16_t speed, int16_t out[NUM_MOTORS]);

/**
 * @brief Compute motor speeds (convenience wrapper)
 * @param dir Movement direction
 * @param speed Base speed (0-255)
 * @return MotorSpeeds structure
 */
MotorSpeeds computeSpeeds(Direction dir, int16_t speed);

/**
 * @brief Apply synchronization correction based on encoder feedback
 *
 * Adjusts individual motor speeds to keep all wheels in sync.
 * Motors that are ahead get slowed down.
 *
 * @param speeds Input/output motor speeds array
 * @param encoderTicks Current encoder readings
 * @param kp Proportional gain for correction
 */
void applySyncCorrection(int16_t speeds[NUM_MOTORS],
                        const int32_t encoderTicks[NUM_MOTORS],
                        float kp);

/**
 * @brief Calculate slowdown speed based on remaining distance
 * @param baseSpeed Target speed
 * @param remainingTicks Distance remaining to target
 * @return Adjusted speed (reduced near target)
 */
int16_t calcSlowdown(int16_t baseSpeed, int32_t remainingTicks);

} // namespace Mecanum

#endif // APP_MECANUM_H
