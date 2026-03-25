/**
 * @file motion.h
 * @brief High-level motion control
 *
 * Manages motion targets, encoder feedback, and per-motor PID velocity control.
 * Each motor has its own PI controller for closed-loop speed tracking.
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
 * Velocity is PID-controlled at 50Hz using encoder feedback.
 */
void setVelocity(Direction dir, int16_t speed);

/**
 * @brief Set per-motor velocity targets (PID-controlled, PWM-scale)
 * @param speeds Array of 4 motor speeds (-255 to 255, PWM-scale)
 *
 * Used for PS2/direction-based velocity control.
 * Speeds are converted to tick-rate targets and tracked by per-motor PID.
 */
void setMotorVelocities(const int16_t speeds[NUM_MOTORS]);

/**
 * @brief Set per-motor tick-rate targets directly (PID-controlled)
 * @param tickRates Array of 4 tick-rate targets (ticks/control-period)
 *
 * Used for VEL command — receives output of computeFromVelocity() directly.
 * No PWM-to-tickrate conversion; values are used as velSetpoint as-is.
 */
void setMotorTickRates(const int16_t tickRates[NUM_MOTORS]);

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
 *   - Per-motor PID velocity control
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
 * @brief Check if motion has stalled (no encoder progress)
 * @return true if motion stalled due to lack of encoder progress
 */
bool isStalled();

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
 * @brief Get motor speeds for direction (utility)
 */
MotorSpeeds getSpeeds(Direction dir, int16_t speed);

//--- Motor Calibration (per-motor speed profiling) ---

enum class CalResult : uint8_t {
    RUNNING,        // Calibration in progress
    DONE,           // Completed successfully
    ABORTED         // Aborted by user (STOP command)
};

/**
 * @brief Start motor calibration sequence
 * Drives all motors forward, measures per-motor tick rates,
 * stores results in EEPROM for PID feed-forward.
 */
void startCalibration();

/**
 * @brief Update calibration state machine - call at 50Hz
 * @return CalResult indicating progress
 */
CalResult updateCalibration();

/**
 * @brief Abort calibration in progress
 */
void abortCalibration();

/**
 * @brief Check if calibration is running
 */
bool isCalibrating();

/**
 * @brief Get per-motor max tick rate (from calibration or default)
 */
const uint8_t* getCalMaxTickrate();

/**
 * @brief Get per-motor reverse max tick rate
 */
const uint8_t* getCalMaxTickrateRev();

/**
 * @brief Get per-motor dead-zone PWM (minimum PWM to start moving)
 */
const uint8_t* getCalDeadZone();

/**
 * @brief Load calibration data from EEPROM
 * @return true if valid data found
 */
bool loadCalibFromEEPROM();

/**
 * @brief Save calibration data to EEPROM
 */
void saveCalibToEEPROM();

} // namespace Motion

#endif // APP_MOTION_H
