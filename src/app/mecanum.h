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
 * @brief Calculate acceleration ramp based on distance traveled
 * @param baseSpeed Target speed
 * @param currentPos Current encoder position (ticks from start)
 * @return Adjusted speed (ramped up from ACCEL_MIN_SPEED)
 */
int16_t calcAccelRamp(int16_t baseSpeed, int32_t currentPos);

/**
 * @brief Calculate slowdown speed based on remaining distance
 * @param baseSpeed Target speed
 * @param remainingTicks Distance remaining to target
 * @return Adjusted speed (reduced near target)
 */
int16_t calcSlowdown(int16_t baseSpeed, int32_t remainingTicks);

/**
 * @brief Inverse kinematics: robot velocity → per-motor tick-rate targets
 * @param vx_mm   Forward velocity in mm/s (positive = forward)
 * @param vy_mm   Lateral velocity in mm/s (positive = left strafe)
 * @param wz_mrad Angular velocity in mrad/s (positive = CCW)
 * @param out     Output: per-motor tick-rate targets (ticks/control-period)
 *
 * Same units as ODOM output — symmetric input/output protocol.
 */
void computeFromVelocity(int16_t vx_mm, int16_t vy_mm, int16_t wz_mrad,
                         int16_t out[NUM_MOTORS]);

/**
 * @brief Forward kinematics: wheel speeds → robot velocity
 * @param wheelDelta Encoder delta ticks per telemetry period (MOVING_TELEMETRY_MS), per motor
 * @param vx_mm     Output: forward velocity in mm/s
 * @param vy_mm     Output: lateral velocity in mm/s (positive = left)
 * @param wz_mrad   Output: angular velocity in mrad/s (positive = CCW)
 */
void forwardKinematics(const int16_t wheelDelta[NUM_MOTORS],
                       int16_t &vx_mm, int16_t &vy_mm, int16_t &wz_mrad);

} // namespace Mecanum

#endif // APP_MECANUM_H
