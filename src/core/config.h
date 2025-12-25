/**
 * @file config.h
 * @brief Centralized configuration for Robot Controller
 *
 * All hardware pins, timing constants, and tuning parameters.
 * Modify this file to adapt to different hardware configurations.
 */

#ifndef CORE_CONFIG_H
#define CORE_CONFIG_H

#include <stdint.h>

//==============================================================================
// Build Configuration
//==============================================================================

// Uncomment to enable debug output
// #define DEBUG_ENABLED
// #define DEBUG_TIMING
// #define DEBUG_MOTORS

//==============================================================================
// Hardware: I2C Motor Shield (PCA9685)
//==============================================================================

#define PCA9685_I2C_ADDR        0x60
#define PCA9685_PWM_FREQ        1600    // Hz for DC motors

// PCA9685 motor pin mapping (TB67H450 H-bridge)
// Each motor uses 2 pins: IN1 (forward PWM) and IN2 (backward PWM)
#define MOTOR1_IN1_PIN          8       // FL Motor
#define MOTOR1_IN2_PIN          9
#define MOTOR2_IN1_PIN          10      // FR Motor
#define MOTOR2_IN2_PIN          11
#define MOTOR3_IN1_PIN          15      // RL Motor
#define MOTOR3_IN2_PIN          14
#define MOTOR4_IN1_PIN          13      // RR Motor
#define MOTOR4_IN2_PIN          12

//==============================================================================
// Hardware: Serial Communication
//==============================================================================

#define SERIAL_BAUD             9600
#define CMD_BUFFER_SIZE         32

//==============================================================================
// Hardware: PS2 Controller
//==============================================================================

#define PS2_DAT_PIN             12
#define PS2_CMD_PIN             11
#define PS2_ATT_PIN             10
#define PS2_CLK_PIN             13
#define PS2_PRESSURES           true
#define PS2_RUMBLE              true
#define PS2_RECONNECT_MS        1000
#define PS2_READ_MS             30

//==============================================================================
// Motor Configuration
//==============================================================================

#define NUM_MOTORS              4

// Motor indices (wheel positions for Mecanum drive)
#define MOTOR_FL                0       // Front-Left
#define MOTOR_FR                1       // Front-Right
#define MOTOR_RL                2       // Rear-Left
#define MOTOR_RR                3       // Rear-Right

// Speed limits
#define SPEED_MIN               20
#define SPEED_MAX               255

// Default motor gains (for calibration compensation)
#define DEFAULT_GAIN_FL         1.00f
#define DEFAULT_GAIN_FR         1.00f
#define DEFAULT_GAIN_RL         1.00f
#define DEFAULT_GAIN_RR         1.00f

//==============================================================================
// Encoder Configuration
//==============================================================================

#define NUM_ENCODERS            4
#define ENCODER_CPR             4320    // Counts per revolution

//==============================================================================
// Control Loop
//==============================================================================

#define CONTROL_FREQ_HZ         50      // 50Hz = 20ms period
#define CONTROL_PERIOD_US       20000

// Timer1 config for 50Hz on 16MHz Arduino
// OCR1A = (16MHz / 256 / 50) - 1 = 1249
#define TIMER1_PRESCALER        256
#define TIMER1_OCR_VALUE        1249

//==============================================================================
// Motion Control
//==============================================================================

// Deceleration ramp
#define SLOWDOWN_TICKS          400     // Start slowing before target
#define SLOWDOWN_KP             0.05f
#define SLOWDOWN_MIN_SPEED      40

// Motor synchronization PID (set to 0 to disable)
#define SYNC_KP                 0.0f
#define SYNC_KI                 0.0f
#define SYNC_KD                 0.0f

// Brake timing
#define BRAKE_HOLD_MS           40

// Timeouts
#define MOVE_TIMEOUT_MS         30000   // Max time for any movement

//==============================================================================
// Calibration
//==============================================================================

#define CAL_SPEED               120
#define CAL_TICKS               3000
#define CAL_SESSIONS            6
#define CAL_SETTLE_MS           150
#define CAL_POST_MS             1000

//==============================================================================
// PS2 Speed Presets
//==============================================================================

#define PS2_SPEED_SLOW          30
#define PS2_SPEED_NORMAL        50
#define PS2_SPEED_FAST          100

//==============================================================================
// Debug Macros
//==============================================================================

#ifdef DEBUG_ENABLED
    #define DBG_PRINT(x)        Serial.print(x)
    #define DBG_PRINTLN(x)      Serial.println(x)
    #define DBG_PRINTF(...)     Serial.printf(__VA_ARGS__)
#else
    #define DBG_PRINT(x)
    #define DBG_PRINTLN(x)
    #define DBG_PRINTF(...)
#endif

#endif // CORE_CONFIG_H
