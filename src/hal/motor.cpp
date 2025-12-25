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
 * PCA9685 channel mapping:
 *   Motor 1 (FL): CH8  (IN1), CH9  (IN2)
 *   Motor 2 (FR): CH10 (IN1), CH11 (IN2)
 *   Motor 3 (RL): CH15 (IN1), CH14 (IN2)
 *   Motor 4 (RR): CH13 (IN1), CH12 (IN2)
 */

#include "motor.h"
#include "pca9685.h"
#include <Arduino.h>
#include <Wire.h>

//==============================================================================
// Motor Pin Mapping
//==============================================================================

namespace {
    // PCA9685 channel for each motor [motor][IN1, IN2]
    const uint8_t MOTOR_CH[NUM_MOTORS][2] = {
        {MOTOR1_IN1_PIN, MOTOR1_IN2_PIN},  // FL: CH8, CH9
        {MOTOR2_IN1_PIN, MOTOR2_IN2_PIN},  // FR: CH10, CH11
        {MOTOR3_IN1_PIN, MOTOR3_IN2_PIN},  // RL: CH15, CH14
        {MOTOR4_IN1_PIN, MOTOR4_IN2_PIN}   // RR: CH13, CH12
    };

    // Current state
    int16_t speeds[NUM_MOTORS] = {0, 0, 0, 0};
    float gains[NUM_MOTORS] = {
        DEFAULT_GAIN_FL,
        DEFAULT_GAIN_FR,
        DEFAULT_GAIN_RL,
        DEFAULT_GAIN_RR
    };

    bool ready = false;

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

        // Apply gain compensation
        float compensated = (float)speed * gains[motor];
        int16_t clamped = constrain((int16_t)compensated, -255, 255);

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
    // Initialize I2C
    Wire.begin();

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

void setGain(uint8_t motor, float gain) {
    if (motor >= NUM_MOTORS) return;
    gains[motor] = constrain(gain, 0.5f, 1.5f);
}

float getGain(uint8_t motor) {
    if (motor >= NUM_MOTORS) return 1.0f;
    return gains[motor];
}

void setAllGains(const float newGains[NUM_MOTORS]) {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        gains[i] = constrain(newGains[i], 0.5f, 1.5f);
    }
}

void getAllGains(float outGains[NUM_MOTORS]) {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        outGains[i] = gains[i];
    }
}

void resetGains() {
    gains[MOTOR_FL] = DEFAULT_GAIN_FL;
    gains[MOTOR_FR] = DEFAULT_GAIN_FR;
    gains[MOTOR_RL] = DEFAULT_GAIN_RL;
    gains[MOTOR_RR] = DEFAULT_GAIN_RR;
}

int16_t getSpeed(uint8_t motor) {
    if (motor >= NUM_MOTORS) return 0;
    return speeds[motor];
}

bool isReady() {
    return ready;
}

} // namespace Motor
