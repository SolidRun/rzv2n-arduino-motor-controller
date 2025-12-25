/**
 * @file main.cpp
 * @brief Mecanum 4WD Robot Controller - Entry Point
 *
 * Minimal main file - all logic is in modules.
 *
 * Architecture:
 *   HAL Layer:  pca9685, motor, encoder, timer
 *   App Layer:  mecanum, motion, serial_cmd, robot
 *
 * Control loop runs at 50Hz via Timer1.
 */

#include <Arduino.h>
#include "robot.h"
#include "../hal/timer.h"

void setup() {
    Robot::init();
}

void loop() {
    // Check for timer tick (50Hz control loop)
    if (Timer::tickPending()) {
        Timer::clearTick();
        Robot::onTick();
    }

    // Handle commands and state machine
    Robot::update();
}
