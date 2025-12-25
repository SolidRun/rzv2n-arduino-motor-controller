/**
 * @file timer.cpp
 * @brief Timer1 HAL Implementation
 *
 * Configures Timer1 in CTC mode for fixed-rate control loop.
 * ISR is minimal - just sets flag, no heavy work.
 *
 * For 50Hz on 16MHz Arduino:
 *   Prescaler = 256
 *   OCR1A = (16000000 / 256 / 50) - 1 = 1249
 */

#include "timer.h"
#include "../core/config.h"
#include <Arduino.h>
#include <avr/interrupt.h>

//==============================================================================
// Private State
//==============================================================================

namespace {
    volatile bool tick = false;
    volatile uint16_t overrunCount = 0;
}

//==============================================================================
// Timer1 ISR
//==============================================================================

ISR(TIMER1_COMPA_vect) {
    if (tick) {
        overrunCount++;  // Previous tick wasn't processed
    }
    tick = true;
}

//==============================================================================
// Public Interface
//==============================================================================

namespace Timer {

void init() {
    // Disable interrupts during setup
    uint8_t sreg = SREG;
    cli();

    // Reset Timer1
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    // Set compare value for desired frequency
    // OCR1A = (F_CPU / prescaler / freq) - 1
    OCR1A = TIMER1_OCR_VALUE;

    // CTC mode (Clear Timer on Compare match)
    TCCR1B |= (1 << WGM12);

    // Prescaler = 256
    TCCR1B |= (1 << CS12);

    // Enable compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    // Clear state
    tick = false;
    overrunCount = 0;

    SREG = sreg;
}

bool tickPending() {
    return tick;
}

void clearTick() {
    tick = false;
}

uint16_t overruns() {
    return overrunCount;
}

void resetOverruns() {
    uint8_t sreg = SREG;
    cli();
    overrunCount = 0;
    SREG = sreg;
}

uint16_t getFrequency() {
    return CONTROL_FREQ_HZ;
}

uint32_t getPeriodUs() {
    return CONTROL_PERIOD_US;
}

} // namespace Timer
