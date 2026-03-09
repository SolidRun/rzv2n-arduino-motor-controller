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
#define DEBUG_ENABLED
// #define DEBUG_TIMING
// #define DEBUG_MOTORS

// Uncomment to include PS2 controller support (~3KB flash)
// #define ENABLE_PS2

//==============================================================================
// Hardware: I2C Motor Shield (PCA9685)
//==============================================================================

#define PCA9685_I2C_ADDR        0x60
#define PCA9685_PWM_FREQ        1600    // Hz for DC motors

// Per-wheel hardware mapping (PCA9685 channels + encoder port)
// To fix a motor/encoder mismatch, change values HERE only.
// TB67H450: IN1 = forward PWM, IN2 = backward PWM
#define FL_IN1_CH               8
#define FL_IN2_CH               9
#define FL_ENC_PORT             1
#define FL_ENC_DIR              1

#define RL_IN1_CH               10
#define RL_IN2_CH               11
#define RL_ENC_PORT             2
#define RL_ENC_DIR              1

#define RR_IN1_CH               15
#define RR_IN2_CH               14
#define RR_ENC_PORT             3
#define RR_ENC_DIR             -1

#define FR_IN1_CH               13
#define FR_IN2_CH               12
#define FR_ENC_PORT             4
#define FR_ENC_DIR             -1

//==============================================================================
// Hardware: Serial Communication
//==============================================================================

#define SERIAL_BAUD             115200
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
// Sequential ordering matches physical wiring:
//   Motor1=FL(idx0), Motor2=RL(idx1), Motor3=RR(idx2), Motor4=FR(idx3)
// All arrays (MOTOR_CH, speeds, encoders) use this indexing.
#define MOTOR_FL                0       // Front-Left  -> Motor1
#define MOTOR_RL                1       // Rear-Left   -> Motor2
#define MOTOR_RR                2       // Rear-Right  -> Motor3
#define MOTOR_FR                3       // Front-Right -> Motor4

// Speed limits
#define SPEED_MIN               20
#define SPEED_MAX               255

//==============================================================================
// Encoder Configuration
//==============================================================================

#define NUM_ENCODERS            4
#define ENCODER_CPR             4320    // Counts per revolution
#define WHEEL_DIAMETER_MM       80      // Mecanum wheel diameter in mm
// Wheel circumference = π × 80mm = 251.3mm
// 1 tick = 0.058mm, 1mm ≈ 17.19 ticks, 1cm ≈ 171.9 ticks

// Robot geometry (for mecanum forward kinematics / odometry)
#define WHEEL_BASE_MM           190     // Front-to-rear axle distance
#define TRACK_WIDTH_MM          210     // Left-to-right wheel distance

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

// Acceleration ramp (prevents wheel slip on startup)
#define ACCEL_RAMP_TICKS        300     // Ramp up over first N encoder ticks
#define ACCEL_MIN_SPEED         40      // Starting speed for ramp

// Deceleration ramp (smooth stopping near target)
#define SLOWDOWN_TICKS          400     // Start slowing before target
#define SLOWDOWN_KP             0.05f
#define SLOWDOWN_MIN_SPEED      60

// Per-motor velocity PID (closed-loop speed control using encoder feedback)
// Each motor has its own PI controller: PWM = feedforward + Kp*error + Ki*integral
// Feed-forward estimates PWM from desired tick rate, PI corrects for load/friction.
#define VEL_PID_KP              1.5f    // Proportional gain (PWM per tick/period error)
#define VEL_PID_KI              0.3f    // Integral gain (corrects steady-state error)
#define VEL_PID_IMAX            150.0f  // Anti-windup: max integral accumulation

// Maximum encoder ticks per control period (20ms) at full PWM (255), no load.
// Motor: 9600 RPM shaft, 90:1 gear → 106.7 RPM output
// 106.7/60 * 4320 CPR = 7680 ticks/s → 7680/50Hz = ~154 ticks/period
#define MAX_MOTOR_TICKRATE      150

// Brake timing
#define BRAKE_HOLD_MS           40

// Motor calibration (per-motor speed profiling)
// Measures each motor's actual max tick rate for accurate PID feed-forward.
#define CALIB_PWM               200     // Reference PWM for measurement
#define CALIB_SETTLE_MS         500     // Wait for motor to reach steady state
#define CALIB_MEASURE_MS        2000    // Measurement window per session
#define CALIB_SESSIONS          3       // Number of measurement sessions (averaged)
#define CALIB_BRAKE_MS          300     // Brake between sessions

// EEPROM layout for calibration data
#define EEPROM_CALIB_ADDR       0       // Start address
#define EEPROM_CALIB_MARKER     0xCB    // Validity marker byte

// Timeouts
#define MOVE_TIMEOUT_MS         30000   // Max time for any movement
#define WATCHDOG_TIMEOUT_MS     200     // Stop motors if no command received
#define STALL_TIMEOUT_MS        5000    // Abort if no encoder progress for this long
#define STALL_MIN_PROGRESS      50     // Minimum ticks needed to reset stall timer

#define MIN_WORK_SPEED 80       
#define SHORT_MOVE_TICKS 250
#define SHORT_REMAIN_TICKS 600

//==============================================================================
// Telemetry & State Machine
//==============================================================================

#define IDLE_TELEMETRY_MS       1000    // Encoder print rate in IDLE state (1Hz)
#define MOVING_TELEMETRY_MS     50      // Encoder print rate while MOVING (20Hz)
#define MOVING_DEBUG_MS         1000    // Motion debug output rate
#define ERROR_RECOVERY_MS       5000    // Auto-recovery time in ERROR state
#define PS2_DISCONNECT_MS       1000    // PS2 disconnect detection threshold

//==============================================================================
// I2C Health Monitoring
//==============================================================================

#define I2C_HEALTH_CHECK_MS     2000    // Check I2C health every 2 seconds
#define I2C_ERROR_THRESHOLD     5       // Enter ERROR state after N consecutive errors

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
#else
    #define DBG_PRINT(x)
    #define DBG_PRINTLN(x)
#endif

#endif // CORE_CONFIG_H