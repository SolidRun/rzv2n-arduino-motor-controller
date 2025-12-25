/**
 * @file motion.h
 * @brief High-level motion control
 *
 * Manages motion targets, encoder feedback, and motor synchronization.
 * Called from the main control loop at fixed rate.
 */

#ifndef APP_MOTION_H
#define APP_MOTION_H

#include <stdint.h>
#include "../core/config.h"
#include "../core/types.h"

namespace Motion {

/**
 * @brief Initialize motion controller
 */
void init();

/**
 * @brief Set motion target
 * @param dir Direction of movement
 * @param speed Speed (0-255)
 * @param ticks Encoder ticks to travel (0 = continuous)
 *
 * If ticks > 0, motion will stop automatically when target reached.
 * If ticks = 0, motion continues until stop() is called.
 */
void setTarget(Direction dir, int16_t speed, int32_t ticks);

/**
 * @brief Set continuous velocity (no encoder target)
 * @param dir Direction of movement
 * @param speed Speed (0-255)
 *
 * Used for PS2 controller input - no automatic stopping.
 */
void setVelocity(Direction dir, int16_t speed);

/**
 * @brief Stop all motion immediately
 * @param brake If true, apply active brake. If false, coast.
 */
void stop(bool brake = true);

/**
 * @brief Clear current target
 * Motors continue at current speed but no longer tracking.
 */
void clearTarget();

/**
 * @brief Update motion control - call at fixed rate
 *
 * Handles:
 *   - Encoder feedback reading
 *   - Target progress checking
 *   - Speed ramping/slowdown
 *   - Motor synchronization
 *
 * Should be called from timer tick handler.
 */
void update();

/**
 * @brief Check if actively moving toward a target
 */
bool isMoving();

/**
 * @brief Check if target has been reached
 * @return true if target reached or no target active
 */
bool isComplete();

/**
 * @brief Get current motion direction
 */
Direction getDirection();

/**
 * @brief Get remaining distance to target
 * @return Ticks remaining, or 0 if no target
 */
int32_t remaining();

/**
 * @brief Run calibration routine (blocking)
 *
 * Measures motor response and adjusts gain compensation
 * to make all wheels travel equal distance.
 */
void calibrate();

/**
 * @brief Get motor speeds for direction (utility)
 */
MotorSpeeds getSpeeds(Direction dir, int16_t speed);

} // namespace Motion

#endif // APP_MOTION_H
