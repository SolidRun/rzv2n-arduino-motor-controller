/**
 * @file pca9685.h
 * @brief PCA9685 PWM Driver HAL
 *
 * Low-level driver for PCA9685 16-channel PWM controller.
 * Used to control TB67H450 H-bridge motor drivers.
 */

#ifndef HAL_PCA9685_H
#define HAL_PCA9685_H

#include <stdint.h>

namespace PCA9685 {

/**
 * @brief Initialize PCA9685
 * @param addr I2C address (default 0x60)
 * @param freq PWM frequency in Hz
 */
void init(uint8_t addr = 0x60, uint16_t freq = 1600);

/**
 * @brief Software reset
 */
void reset();

/**
 * @brief Set single channel PWM
 * @param channel Channel 0-15
 * @param on ON time (0-4095, or 4096 for always on)
 * @param off OFF time (0-4095, or 4096 for always off)
 */
void setPWM(uint8_t channel, uint16_t on, uint16_t off);

/**
 * @brief Set channel to full ON
 * @param channel Channel 0-15
 */
void setOn(uint8_t channel);

/**
 * @brief Set channel to full OFF
 * @param channel Channel 0-15
 */
void setOff(uint8_t channel);

/**
 * @brief Set channel duty cycle (0-4095)
 * @param channel Channel 0-15
 * @param duty Duty cycle 0-4095
 */
void setDuty(uint8_t channel, uint16_t duty);

/**
 * @brief Update multiple channels efficiently
 * @param startChannel Starting channel
 * @param count Number of channels
 * @param duties Array of duty values
 */
void setMultiple(uint8_t startChannel, uint8_t count, const uint16_t* duties);

/**
 * @brief Get accumulated I2C error count
 * @return Number of I2C errors since last reset
 */
uint16_t getI2CErrors();

/**
 * @brief Reset I2C error counter
 */
void resetI2CErrors();

/**
 * @brief Check if PCA9685 is responding
 * @return true if device is initialized and responding
 */
bool isHealthy();

} // namespace PCA9685

#endif // HAL_PCA9685_H
