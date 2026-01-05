/**
 * @file pca9685.cpp
 * @brief PCA9685 PWM Driver HAL Implementation
 *
 * Low-level I2C driver for PCA9685 16-channel, 12-bit PWM controller.
 * Handles register access, frequency configuration, and PWM output.
 */

#include "pca9685.h"
#include <Arduino.h>
#include <Wire.h>

//==============================================================================
// PCA9685 Register Definitions
//==============================================================================

#define PCA9685_MODE1       0x00
#define PCA9685_MODE2       0x01
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06
#define PCA9685_LED0_ON_H   0x07
#define PCA9685_LED0_OFF_L  0x08
#define PCA9685_LED0_OFF_H  0x09
#define PCA9685_ALL_ON_L    0xFA
#define PCA9685_ALL_ON_H    0xFB
#define PCA9685_ALL_OFF_L   0xFC
#define PCA9685_ALL_OFF_H   0xFD

// MODE1 bits
#define MODE1_RESTART       0x80
#define MODE1_SLEEP         0x10
#define MODE1_AI            0x20    // Auto-increment
#define MODE1_ALLCALL       0x01

// MODE2 bits
#define MODE2_OUTDRV        0x04    // Totem-pole output

// Special PWM values
#define PWM_FULL_ON         0x1000  // Bit 12 = always ON
#define PWM_FULL_OFF        0x1000  // Bit 12 in OFF register = always OFF

//==============================================================================
// Private State
//==============================================================================

namespace {
    uint8_t i2cAddr = 0x60;
    bool initialized = false;
    uint16_t i2cErrors = 0;     // Count I2C errors for diagnostics
    uint8_t lastError = 0;      // Last I2C error code

    /**
     * @brief Write single byte to register with error handling
     * @return true if successful, false on I2C error
     */
    bool writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(i2cAddr);
        Wire.write(reg);
        Wire.write(val);
        uint8_t err = Wire.endTransmission();
        if (err != 0) {
            lastError = err;
            i2cErrors++;
            return false;
        }
        return true;
    }

    /**
     * @brief Read single byte from register
     * @return register value, or 0xFF on error
     */
    uint8_t readReg(uint8_t reg) {
        Wire.beginTransmission(i2cAddr);
        Wire.write(reg);
        uint8_t err = Wire.endTransmission();
        if (err != 0) {
            lastError = err;
            i2cErrors++;
            return 0xFF;
        }
        if (Wire.requestFrom(i2cAddr, (uint8_t)1) != 1) {
            i2cErrors++;
            return 0xFF;
        }
        return Wire.read();
    }
}

//==============================================================================
// Public Interface Implementation
//==============================================================================

namespace PCA9685 {

void init(uint8_t addr, uint16_t freq) {
    i2cAddr = addr;

    // Reset device
    reset();

    // Set MODE2: totem-pole outputs
    writeReg(PCA9685_MODE2, MODE2_OUTDRV);

    // Set MODE1: auto-increment enabled
    writeReg(PCA9685_MODE1, MODE1_AI);

    // Wait for oscillator
    delayMicroseconds(500);

    // Set PWM frequency
    // Formula: prescale = round(25MHz / (4096 * freq)) - 1
    // With correction factor from Adafruit library
    float prescaleval = 25000000.0f;
    prescaleval /= 4096.0f;
    prescaleval /= (float)freq;
    prescaleval -= 1.0f;
    prescaleval *= 0.9f;  // Correction factor

    uint8_t prescale = (uint8_t)(prescaleval + 0.5f);

    // Must be in sleep mode to set prescale
    uint8_t oldmode = readReg(PCA9685_MODE1);
    writeReg(PCA9685_MODE1, (oldmode & ~MODE1_RESTART) | MODE1_SLEEP);
    writeReg(PCA9685_PRESCALE, prescale);
    writeReg(PCA9685_MODE1, oldmode);

    delayMicroseconds(500);

    // Enable restart
    writeReg(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);

    // All channels off initially
    writeReg(PCA9685_ALL_ON_L, 0);
    writeReg(PCA9685_ALL_ON_H, 0);
    writeReg(PCA9685_ALL_OFF_L, 0);
    writeReg(PCA9685_ALL_OFF_H, PWM_FULL_OFF >> 8);  // Bit 4 = full off

    initialized = true;
}

void reset() {
    // Software reset via general call address
    Wire.beginTransmission(0x00);
    Wire.write(0x06);  // SWRST command
    Wire.endTransmission();
    delay(10);
}

void setPWM(uint8_t channel, uint16_t on, uint16_t off) {
    if (!initialized || channel > 15) return;

    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;

    Wire.beginTransmission(i2cAddr);
    Wire.write(reg);
    Wire.write(on & 0xFF);         // ON_L
    Wire.write(on >> 8);           // ON_H
    Wire.write(off & 0xFF);        // OFF_L
    Wire.write(off >> 8);          // OFF_H
    Wire.endTransmission();
}

void setOn(uint8_t channel) {
    if (!initialized || channel > 15) return;
    setPWM(channel, PWM_FULL_ON, 0);
}

void setOff(uint8_t channel) {
    if (!initialized || channel > 15) return;
    setPWM(channel, 0, PWM_FULL_OFF);
}

void setDuty(uint8_t channel, uint16_t duty) {
    if (!initialized || channel > 15) return;

    if (duty == 0) {
        setOff(channel);
    } else if (duty >= 4095) {
        setOn(channel);
    } else {
        setPWM(channel, 0, duty);
    }
}

void setMultiple(uint8_t startChannel, uint8_t count, const uint16_t* duties) {
    if (!initialized || startChannel > 15) return;
    if (startChannel + count > 16) count = 16 - startChannel;

    // Arduino Wire buffer is 32 bytes
    // Each channel needs 4 bytes, so max 7 channels per transaction
    // Split into smaller transactions if needed

    uint8_t remaining = count;
    uint8_t offset = 0;

    while (remaining > 0) {
        uint8_t batch = (remaining > 6) ? 6 : remaining;  // 1 addr + 24 data = 25 bytes
        uint8_t reg = PCA9685_LED0_ON_L + 4 * (startChannel + offset);

        Wire.beginTransmission(i2cAddr);
        Wire.write(reg);

        for (uint8_t i = 0; i < batch; i++) {
            uint16_t duty = duties[offset + i];
            uint16_t on, off;

            if (duty == 0) {
                on = 0;
                off = PWM_FULL_OFF;
            } else if (duty >= 4095) {
                on = PWM_FULL_ON;
                off = 0;
            } else {
                on = 0;
                off = duty;
            }

            Wire.write(on & 0xFF);
            Wire.write(on >> 8);
            Wire.write(off & 0xFF);
            Wire.write(off >> 8);
        }

        uint8_t err = Wire.endTransmission();
        if (err != 0) {
            lastError = err;
            i2cErrors++;
        }

        remaining -= batch;
        offset += batch;
    }
}

uint16_t getI2CErrors() {
    return i2cErrors;
}

void resetI2CErrors() {
    i2cErrors = 0;
    lastError = 0;
}

bool isHealthy() {
    // Check if we can communicate with the device
    Wire.beginTransmission(i2cAddr);
    return (Wire.endTransmission() == 0) && initialized;
}

} // namespace PCA9685
