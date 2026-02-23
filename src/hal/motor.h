/**
 * @file motor.h
 * @brief Motor HAL for TB67H450 H-bridge via PCA9685
 *
 * Provides motor control abstraction for 4-wheel mecanum drive.
 * Each motor uses 2 PWM channels for H-bridge control.
 */

#ifndef HAL_MOTOR_H
#define HAL_MOTOR_H

#include <stdint.h>
#include "../core/config.h"

namespace Motor {

/**
 * @brief Initialize motor driver hardware
 * Initializes PCA9685 and sets all motors to coast
 */
void init();

/**
 * @brief Set single motor speed
 * @param motor Motor index (0-3: FL, FR, RL, RR)
 * @param speed Speed -255 to +255 (sign = direction)
 */
void set(uint8_t motor, int16_t speed);

/**
 * @brief Set single motor speed directly (bypasses batching)
 * @param motor Motor index (0-3)
 * @param speed Raw PWM -255 to +255
 * Used for diagnostics: tests the raw motor hardware without PID.
 */
void setDirect(uint8_t motor, int16_t speed);

/**
 * @brief Set all motor speeds atomically
 * @param speeds Array of 4 speeds (FL, FR, RL, RR)
 */
void setAll(const int16_t speeds[NUM_MOTORS]);

/**
 * @brief Active brake on single motor
 * @param motor Motor index (0-3)
 */
void brake(uint8_t motor);

/**
 * @brief Active brake on all motors
 */
void brakeAll();

/**
 * @brief Coast (release) single motor
 * @param motor Motor index (0-3)
 */
void coast(uint8_t motor);

/**
 * @brief Coast (release) all motors
 */
void coastAll();

/**
 * @brief Emergency stop - brake all motors immediately
 */
void emergencyStop();

/**
 * @brief Brake then coast after delay (BLOCKING - use startBrake for non-blocking)
 * @param brakeMs Duration to hold brake in milliseconds
 */
void brakeAndRelease(uint16_t brakeMs);

/**
 * @brief Start a timed brake (non-blocking)
 * @param ms Duration to hold brake before auto-releasing to coast
 * Call updateBrake() from main loop to service the timer.
 */
void startBrake(uint16_t ms);

/**
 * @brief Service the brake timer - call from main loop
 * Releases to coast when brake duration expires.
 */
void updateBrake();

/**
 * @brief Check if a timed brake is currently active
 */
bool isBraking();

/**
 * @brief Get current speed setting for motor
 * @param motor Motor index (0-3)
 * @return Last set speed value
 */
int16_t getSpeed(uint8_t motor);

/**
 * @brief Check if motor driver is initialized
 */
bool isReady();

/**
 * @brief Check I2C communication health
 * @return true if PCA9685 is responding
 */
bool checkHealth();

/**
 * @brief Get accumulated I2C error count
 */
uint16_t getI2CErrorCount();

} // namespace Motor

#endif // HAL_MOTOR_H
