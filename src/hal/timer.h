/**
 * @file timer.h
 * @brief Timer HAL for control loop timing
 *
 * Configures Timer1 for fixed-rate control loop.
 * ISR only sets a flag - actual work done in main loop.
 */

#ifndef HAL_TIMER_H
#define HAL_TIMER_H

#include <stdint.h>

namespace Timer {

/**
 * @brief Initialize and start Timer1 for control loop
 * Configures CTC mode at CONTROL_FREQ_HZ (default 50Hz)
 */
void init();

/**
 * @brief Check if a tick is pending
 * @return true if timer has fired since last clear
 */
bool tickPending();

/**
 * @brief Clear the pending tick flag
 * Call this after processing a tick
 */
void clearTick();

/**
 * @brief Get number of timer overruns
 * Overrun = new tick before previous was cleared
 * @return Cumulative overrun count
 */
uint16_t overruns();

/**
 * @brief Reset overrun counter
 */
void resetOverruns();

/**
 * @brief Get control loop frequency
 * @return Frequency in Hz
 */
uint16_t getFrequency();

/**
 * @brief Get control loop period
 * @return Period in microseconds
 */
uint32_t getPeriodUs();

} // namespace Timer

#endif // HAL_TIMER_H
