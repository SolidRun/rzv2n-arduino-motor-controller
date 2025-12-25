/**
 * @file robot.h
 * @brief Robot state machine
 *
 * Top-level state machine coordinating:
 *   - Serial command processing
 *   - PS2 controller input (optional)
 *   - Motion control
 *   - State transitions
 */

#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#include <stdint.h>
#include "../core/types.h"

namespace Robot {

/**
 * @brief Initialize robot state machine
 */
void init();

/**
 * @brief Main update - call from main loop
 *
 * Handles:
 *   - Serial command processing
 *   - PS2 controller polling
 *   - Motion state checking
 *   - State transitions
 */
void update();

/**
 * @brief Timer tick handler - call at fixed rate
 *
 * Updates motion controller when tick fires.
 */
void onTick();

/**
 * @brief Get current state
 */
State getState();

/**
 * @brief Get state name for debugging
 */
const char* getStateName();

/**
 * @brief Emergency stop - halt all motors
 */
void emergencyStop();

/**
 * @brief Get time in current state (ms)
 */
uint32_t stateTime();

/**
 * @brief Check if PS2 controller is active
 */
bool isPS2Active();

} // namespace Robot

#endif // APP_ROBOT_H
