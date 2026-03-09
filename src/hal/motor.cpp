/**
 * @file motor.cpp
 * @brief Motor HAL Implementation for TB67H450 via PCA9685
 *
 * TB67H450 H-bridge control modes:
 *   Forward:  IN1=PWM, IN2=LOW  (PWM on IN1)
 *   Backward: IN1=LOW, IN2=PWM  (PWM on IN2)
 *   Brake:    IN1=HIGH, IN2=HIGH (both full on - shorts motor)
 *   Coast:    IN1=LOW, IN2=LOW  (both off - freewheel)
 *
 * PCA9685 channel mapping (physical motor -> PCA9685 channels):
 *   Motor 1 (FL): CH8  (IN1), CH9  (IN2) -> MOTOR_FL = index 0
 *   Motor 2 (RL): CH10 (IN1), CH11 (IN2) -> MOTOR_RL = index 1
 *   Motor 3 (RR): CH15 (IN1), CH14 (IN2) -> MOTOR_RR = index 2
 *   Motor 4 (FR): CH13 (IN1), CH12 (IN2) -> MOTOR_FR = index 3
 */

#include "motor.h"
#include "pca9685.h"
#include <Arduino.h>
#include <Wire.h>

//==============================================================================
// Motor Pin Mapping
//==============================================================================

namespace {
    // PCA9685 channel for each motor [motor_index][IN1, IN2]
    // Indexed by MOTOR_FL=0, MOTOR_RL=1, MOTOR_RR=2, MOTOR_FR=3
    const uint8_t MOTOR_CH[NUM_MOTORS][2] = {
        {FL_IN1_CH, FL_IN2_CH},  // [0] MOTOR_FL
        {RL_IN1_CH, RL_IN2_CH},  // [1] MOTOR_RL
        {RR_IN1_CH, RR_IN2_CH},  // [2] MOTOR_RR
        {FR_IN1_CH, FR_IN2_CH}   // [3] MOTOR_FR
    };

    // Current state (indexed by MOTOR_FL/RL/RR/FR)
    int16_t speeds[NUM_MOTORS] = {0, 0, 0, 0};

    bool ready = false;

    // Non-blocking brake state
    bool braking = false;
    uint32_t brakeStart = 0;
    uint16_t brakeDuration = 0;

    // PWM buffer for batch updates (channels 8-15)
    uint16_t pwmBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    bool bufferDirty = false;

    /**
     * @brief Convert motor/pin to buffer index
     */
    inline uint8_t chToIdx(uint8_t channel) {
        return channel - 8;  // Channels 8-15 map to indices 0-7
    }

    /**
     * @brief Set buffer entry to OFF (0V)
     */
    inline void bufferOff(uint8_t channel) {
        pwmBuffer[chToIdx(channel)] = 0;
        bufferDirty = true;
    }

    /**
     * @brief Set buffer entry to ON (full voltage)
     */
    inline void bufferOn(uint8_t channel) {
        pwmBuffer[chToIdx(channel)] = 4095;
        bufferDirty = true;
    }

    /**
     * @brief Set buffer entry to PWM duty cycle
     */
    inline void bufferPWM(uint8_t channel, uint16_t duty) {
        pwmBuffer[chToIdx(channel)] = duty;
        bufferDirty = true;
    }

    /**
     * @brief Flush buffer to PCA9685
     */
    void flushBuffer() {
        if (!bufferDirty) return;

        // Update all 8 channels (8-15) at once
        PCA9685::setMultiple(8, 8, pwmBuffer);
        bufferDirty = false;
    }

    /**
     * @brief Update buffer for single motor
     */
    void updateMotorBuffer(uint8_t motor, int16_t speed) {
        if (motor >= NUM_MOTORS) return;

        uint8_t in1 = MOTOR_CH[motor][0];
        uint8_t in2 = MOTOR_CH[motor][1];

        int16_t clamped = constrain(speed, (int16_t)-255, (int16_t)255);

        // Convert 0-255 to 0-4095 (12-bit PWM)
        uint16_t duty = ((uint16_t)abs(clamped)) << 4;  // * 16

        if (clamped > 0) {
            // Forward: IN1=PWM, IN2=LOW
            bufferOff(in2);
            bufferPWM(in1, duty);
        } else if (clamped < 0) {
            // Backward: IN1=LOW, IN2=PWM
            bufferOff(in1);
            bufferPWM(in2, duty);
        } else {
            // Coast: both LOW
            bufferOff(in1);
            bufferOff(in2);
        }

        speeds[motor] = speed;
    }

    /**
     * @brief Set motor for brake in buffer
     */
    void brakeMotorBuffer(uint8_t motor) {
        if (motor >= NUM_MOTORS) return;

        uint8_t in1 = MOTOR_CH[motor][0];
        uint8_t in2 = MOTOR_CH[motor][1];

        // Brake: both HIGH
        bufferOn(in1);
        bufferOn(in2);
        speeds[motor] = 0;
    }

    /**
     * @brief Set motor for coast in buffer
     */
    void coastMotorBuffer(uint8_t motor) {
        if (motor >= NUM_MOTORS) return;

        uint8_t in1 = MOTOR_CH[motor][0];
        uint8_t in2 = MOTOR_CH[motor][1];

        // Coast: both LOW
        bufferOff(in1);
        bufferOff(in2);
        speeds[motor] = 0;
    }
}

//==============================================================================
// Public Interface
//==============================================================================

namespace Motor {

void init() {
    // Initialize I2C with timeout protection
    Wire.begin();
    Wire.setWireTimeout(3000, true);  // 3ms timeout, reset bus on hang

    // Initialize PCA9685
    PCA9685::init(PCA9685_I2C_ADDR, PCA9685_PWM_FREQ);

    // All motors to coast
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        coastMotorBuffer(i);
    }
    flushBuffer();

    ready = true;
}

void set(uint8_t motor, int16_t speed) {
    if (!ready || motor >= NUM_MOTORS) return;

    if (speeds[motor] != speed) {
        updateMotorBuffer(motor, speed);
        flushBuffer();
    }
}

void setDirect(uint8_t motor, int16_t speed) {
    if (!ready || motor >= NUM_MOTORS) return;

    uint8_t in1 = MOTOR_CH[motor][0];
    uint8_t in2 = MOTOR_CH[motor][1];

    int16_t clamped = constrain(speed, (int16_t)-255, (int16_t)255);
    uint16_t duty = ((uint16_t)abs(clamped)) << 4;  // 0-255 -> 0-4080

    if (clamped > 0) {
        bufferOff(in2);
        bufferPWM(in1, duty);
    } else if (clamped < 0) {
        bufferOff(in1);
        bufferPWM(in2, duty);
    } else {
        bufferOff(in1);
        bufferOff(in2);
    }

    speeds[motor] = speed;
    flushBuffer();
}

void setAll(const int16_t newSpeeds[NUM_MOTORS]) {
    if (!ready) return;

    bool needsUpdate = false;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (speeds[i] != newSpeeds[i]) {
            updateMotorBuffer(i, newSpeeds[i]);
            needsUpdate = true;
        }
    }

    if (needsUpdate) {
        flushBuffer();
    }
}

void brake(uint8_t motor) {
    if (!ready || motor >= NUM_MOTORS) return;

    brakeMotorBuffer(motor);
    flushBuffer();
}

void brakeAll() {
    if (!ready) return;

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        brakeMotorBuffer(i);
    }
    flushBuffer();
}

void coast(uint8_t motor) {
    if (!ready || motor >= NUM_MOTORS) return;

    coastMotorBuffer(motor);
    flushBuffer();
}

void coastAll() {
    if (!ready) return;

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        coastMotorBuffer(i);
    }
    flushBuffer();
}

void emergencyStop() {
    brakeAll();
}

void brakeAndRelease(uint16_t brakeMs) {
    brakeAll();
    delay(brakeMs);
    coastAll();
}

void startBrake(uint16_t ms) {
    brakeAll();
    braking = true;
    brakeStart = millis();
    brakeDuration = ms;
}

void updateBrake() {
    if (braking && (millis() - brakeStart >= brakeDuration)) {
        coastAll();
        braking = false;
    }
}

bool isBraking() {
    return braking;
}

int16_t getSpeed(uint8_t motor) {
    if (motor >= NUM_MOTORS) return 0;
    return speeds[motor];
}

bool isReady() {
    return ready;
}

bool checkHealth() {
    return PCA9685::isHealthy();
}

uint16_t getI2CErrorCount() {
    return PCA9685::getI2CErrors();
}

} // namespace Motor
