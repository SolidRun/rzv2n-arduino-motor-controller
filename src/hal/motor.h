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
 * @brief Brake then coast after delay
 * @param brakeMs Duration to hold brake in milliseconds
 */
void brakeAndRelease(uint16_t brakeMs);

/**
 * @brief Set motor gain compensation
 * @param motor Motor index (0-3)
 * @param gain Gain factor (0.5 to 1.5)
 */
void setGain(uint8_t motor, float gain);

/**
 * @brief Get motor gain compensation
 * @param motor Motor index (0-3)
 * @return Gain factor
 */
float getGain(uint8_t motor);

/**
 * @brief Set all motor gains
 * @param gains Array of 4 gain values
 */
void setAllGains(const float gains[NUM_MOTORS]);

/**
 * @brief Get all motor gains
 * @param gains Output array for 4 gain values
 */
void getAllGains(float gains[NUM_MOTORS]);

/**
 * @brief Reset all gains to defaults
 */
void resetGains();

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

} // namespace Motor

#endif // HAL_MOTOR_H
