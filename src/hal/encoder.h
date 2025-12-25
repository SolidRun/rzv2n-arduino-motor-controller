/**
 * @file encoder.h
 * @brief Encoder HAL with atomic access
 *
 * Provides ISR-safe access to quadrature encoders.
 * Uses QGPMaker_Encoder library for pin change interrupt handling.
 */

#ifndef HAL_ENCODER_H
#define HAL_ENCODER_H

#include <stdint.h>
#include "../core/config.h"

namespace Encoder {

/**
 * @brief Snapshot of all encoder readings at one instant
 */
struct Snapshot {
    int32_t ticks[NUM_ENCODERS];
    uint32_t timestamp_us;

    /** @brief Get average absolute position */
    int32_t average() const;

    /** @brief Get minimum absolute position */
    int32_t minVal() const;

    /** @brief Get maximum absolute position */
    int32_t maxVal() const;

    /** @brief Check if all encoders reached target */
    bool allReached(int32_t target) const;
};

/**
 * @brief Initialize encoder hardware
 */
void init();

/**
 * @brief Read single encoder (atomic on 8-bit MCU)
 * @param index Encoder index (0-3)
 * @return Current tick count (can be negative)
 */
int32_t read(uint8_t index);

/**
 * @brief Read absolute value of encoder
 * @param index Encoder index (0-3)
 * @return Absolute tick count
 */
int32_t readAbs(uint8_t index);

/**
 * @brief Get atomic snapshot of all encoders
 * @param out Output snapshot structure
 */
void getSnapshot(Snapshot& out);

/**
 * @brief Reset single encoder to zero
 * @param index Encoder index (0-3)
 */
void reset(uint8_t index);

/**
 * @brief Reset all encoders to zero
 */
void resetAll();

/**
 * @brief Check if all encoders reached target
 * @param target Target tick count (absolute value compared)
 * @return true if all encoders >= target
 */
bool allReached(int32_t target);

/**
 * @brief Get average absolute position
 * @return Average of all encoder absolute values
 */
int32_t average();

/**
 * @brief Get minimum absolute position
 */
int32_t minVal();

/**
 * @brief Get maximum absolute position
 */
int32_t maxVal();

} // namespace Encoder

#endif // HAL_ENCODER_H
