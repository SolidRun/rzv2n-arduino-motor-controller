/**
 * @file types.h
 * @brief Common type definitions for Robot Controller
 *
 * All enums, structs, and utility functions used across modules.
 */

#ifndef CORE_TYPES_H
#define CORE_TYPES_H

#include <stdint.h>
#include "config.h"

//==============================================================================
// Enumerations
//==============================================================================

/**
 * @brief Movement directions for Mecanum drive
 */
enum class Direction : uint8_t {
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,           // Strafe left
    RIGHT,          // Strafe right
    ROTATE_CW,      // Rotate clockwise
    ROTATE_CCW,     // Rotate counter-clockwise
    DIAG_FL,        // Diagonal forward-left
    DIAG_FR,        // Diagonal forward-right
    DIAG_BL,        // Diagonal backward-left
    DIAG_BR         // Diagonal backward-right
};

/**
 * @brief Robot state machine states
 */
enum class State : uint8_t {
    IDLE,           // Ready for commands
    MOVING,         // Executing movement
    CALIBRATING,    // Running calibration
    ERROR           // Error state
};

/**
 * @brief Serial command types
 */
enum class CmdType : uint8_t {
    NONE,
    STOP,
    READ_ENCODERS,
    CALIBRATE,
    MOVE,           // FWD/BWD/TURN with speed and ticks
    TWIST, 
    UNKNOWN
};

/**
 * @brief Input source
 */
enum class InputSource : uint8_t {
    INPUT_NONE,
    INPUT_SERIAL,
    INPUT_PS2
};

//==============================================================================
// Data Structures
//==============================================================================

/**
 * @brief Motor speeds for all 4 motors
 */
struct MotorSpeeds {
    int16_t m[NUM_MOTORS];  // -255 to +255 (sign = direction)

    void clear() {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) m[i] = 0;
    }
};

/**
 * @brief Encoder readings snapshot
 */
struct EncoderData {
    int32_t ticks[NUM_ENCODERS];
    uint32_t timestamp;

    int32_t average() const {
        int32_t sum = 0;
        for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
            sum += (ticks[i] < 0) ? -ticks[i] : ticks[i];
        }
        return sum / NUM_ENCODERS;
    }

    int32_t minVal() const {
        int32_t val = (ticks[0] < 0) ? -ticks[0] : ticks[0];
        for (uint8_t i = 1; i < NUM_ENCODERS; i++) {
            int32_t absv = (ticks[i] < 0) ? -ticks[i] : ticks[i];
            if (absv < val) val = absv;
        }
        return val;
    }

    bool allReached(int32_t target) const {
        if (target <= 0) return false;
        for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
            int32_t absv = (ticks[i] < 0) ? -ticks[i] : ticks[i];
            if (absv < target) return false;
        }
        return true;
    }
};

/**
 * @brief Motion target
 */
struct MotionTarget {
    Direction dir;
    int16_t speed;
    int32_t ticks;
    bool active;

    void clear() {
        dir = Direction::STOP;
        speed = 0;
        ticks = 0;
        active = false;
    }
};

/**
 * @brief Parsed command from serial
 */
struct Command {
    CmdType type;
    Direction dir;
    int16_t speed;
    int32_t ticks;

    // Velocity streaming (fixed-point)
    int32_t v_mmps;     // mm/s
    int32_t w_mradps;   // mrad/s
};

/**
 * @brief Motor gain calibration
 */
struct MotorGains {
    float g[NUM_MOTORS];

    void reset() {
        g[MOTOR_FL] = DEFAULT_GAIN_FL;
        g[MOTOR_FR] = DEFAULT_GAIN_FR;
        g[MOTOR_RL] = DEFAULT_GAIN_RL;
        g[MOTOR_RR] = DEFAULT_GAIN_RR;
    }

    void normalize() {
        float maxG = g[0];
        for (uint8_t i = 1; i < NUM_MOTORS; i++) {
            if (g[i] > maxG) maxG = g[i];
        }
        if (maxG > 0) {
            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                g[i] /= maxG;
            }
        }
    }
};

//==============================================================================
// Utility Functions
//==============================================================================

/**
 * @brief Clamp value to range
 */
template<typename T>
inline T clamp(T val, T lo, T hi) {
    return (val < lo) ? lo : ((val > hi) ? hi : val);
}

/**
 * @brief Absolute value
 */
template<typename T>
inline T absVal(T val) {
    return (val < 0) ? -val : val;
}

/**
 * @brief Sign of value (-1, 0, +1)
 */
template<typename T>
inline int8_t sign(T val) {
    return (val > 0) ? 1 : ((val < 0) ? -1 : 0);
}

#endif // CORE_TYPES_H
