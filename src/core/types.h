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
    CALIBRATING,    // Motor speed profiling in progress
    TESTING,        // Diagnostic test mode (TMOTOR/TENC)
    ERROR           // Error state
};

/**
 * @brief Serial command types
 */
enum class CmdType : uint8_t {
    NONE,
    STOP,
    READ_ENCODERS,
    MOVE,           // FWD/BWD/TURN with speed and ticks
    VELOCITY,       // VEL,vx,vy,wz continuous velocity
    CALIBRATE,      // CALIB - per-motor speed profiling
    TEST_MOTOR,     // TMOTOR,index,pwm - raw single motor test
    TEST_ENCODER,   // TENC - encoder monitoring mode
    UNKNOWN
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
    int16_t vx, vy, wz;      // For VELOCITY command
    uint8_t motorIndex;       // For TEST_MOTOR command (0-3)
    int16_t pwm;              // For TEST_MOTOR command (-255 to 255)
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
