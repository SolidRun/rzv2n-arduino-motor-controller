/**
 * @file encoder.cpp
 * @brief Encoder HAL Implementation
 *
 * Uses QGPMaker_Encoder library which handles pin change interrupts.
 * All reads are made atomic for ISR safety on 8-bit AVR.
 */

#include "encoder.h"
#include <Arduino.h>

// Include the encoder library from lib folder
#include "QGPMaker_Encoder.h"

//==============================================================================
// Private State
//==============================================================================

namespace {
    // Encoder objects (QGPMaker library handles interrupt setup)
    QGPMaker_Encoder encoders[NUM_ENCODERS] = {
        QGPMaker_Encoder(1),
        QGPMaker_Encoder(2),
        QGPMaker_Encoder(3),
        QGPMaker_Encoder(4)
    };
}

//==============================================================================
// Snapshot Methods
//==============================================================================

int32_t Encoder::Snapshot::average() const {
    int32_t sum = 0;
    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
        int32_t absv = (ticks[i] < 0) ? -ticks[i] : ticks[i];
        sum += absv;
    }
    return sum / NUM_ENCODERS;
}

int32_t Encoder::Snapshot::minVal() const {
    int32_t result = (ticks[0] < 0) ? -ticks[0] : ticks[0];
    for (uint8_t i = 1; i < NUM_ENCODERS; i++) {
        int32_t absv = (ticks[i] < 0) ? -ticks[i] : ticks[i];
        if (absv < result) result = absv;
    }
    return result;
}

int32_t Encoder::Snapshot::maxVal() const {
    int32_t result = (ticks[0] < 0) ? -ticks[0] : ticks[0];
    for (uint8_t i = 1; i < NUM_ENCODERS; i++) {
        int32_t absv = (ticks[i] < 0) ? -ticks[i] : ticks[i];
        if (absv > result) result = absv;
    }
    return result;
}

bool Encoder::Snapshot::allReached(int32_t target) const {
    if (target <= 0) return false;

    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
        int32_t absv = (ticks[i] < 0) ? -ticks[i] : ticks[i];
        if (absv < target) return false;
    }
    return true;
}

//==============================================================================
// Public Interface
//==============================================================================

namespace Encoder {

void init() {
    // Encoders are initialized by constructors
    // Just reset to start at zero
    resetAll();
}

int32_t read(uint8_t index) {
    if (index >= NUM_ENCODERS) return 0;

    int32_t value;
    uint8_t sreg = SREG;
    cli();  // Disable interrupts for atomic 32-bit read
    value = encoders[index].read();
    SREG = sreg;

    return value;
}

int32_t readAbs(uint8_t index) {
    int32_t value = read(index);
    return (value < 0) ? -value : value;
}

void getSnapshot(Snapshot& out) {
    uint8_t sreg = SREG;
    cli();  // Disable interrupts for consistent snapshot

    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
        out.ticks[i] = encoders[i].read();
    }
    out.timestamp_us = micros();

    SREG = sreg;
}

void reset(uint8_t index) {
    if (index >= NUM_ENCODERS) return;

    uint8_t sreg = SREG;
    cli();
    encoders[index].write(0);
    SREG = sreg;
}

void resetAll() {
    uint8_t sreg = SREG;
    cli();

    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
        encoders[i].write(0);
    }
    SREG = sreg;
    Serial.println(F("ENC_RESET"));
}

bool allReached(int32_t target) {
    if (target <= 0) return false;

    Snapshot snap;
    getSnapshot(snap);
    return snap.allReached(target);
}

int32_t average() {
    Snapshot snap;
    getSnapshot(snap);
    return snap.average();
}

int32_t minVal() {
    Snapshot snap;
    getSnapshot(snap);
    return snap.minVal();
}

int32_t maxVal() {
    Snapshot snap;
    getSnapshot(snap);
    return snap.maxVal();
}

} // namespace Encoder
