/**
 * @file serial_cmd.h
 * @brief Serial command interface
 *
 * Non-blocking command parser for robot control.
 * Protocol: CMD,param1,param2\n
 *
 * Commands:
 *   STOP           - Emergency stop
 *   READ           - Read encoder values
 *   FWD,s,t        - Forward: speed s, ticks t
 *   BWD,s,t        - Backward: speed s, ticks t
 *   TURN,s,t       - Turn: speed s, ticks t (positive=left, negative=right)
 *   VEL,vx,vy,wz   - Continuous velocity: forward, strafe, rotation (-255..255)
 *   CALIB           - Calibrate motors: per-motor speed profiling, saves to EEPROM
 *   TMOTOR,idx,pwm  - Test single motor: index (0-3 or FL/FR/RL/RR), PWM (-255..255)
 *   TENC            - Test encoders: stream readings, spin wheels manually
 */

#ifndef APP_SERIAL_CMD_H
#define APP_SERIAL_CMD_H

#include <stdint.h>
#include "../core/config.h"
#include "../core/types.h"

namespace Serial_Cmd {

/**
 * @brief Initialize serial and command parser
 */
void init();

/**
 * @brief Update parser - read available serial data
 * Call from main loop (non-blocking)
 */
void update();

/**
 * @brief Check if command is ready
 */
bool hasCommand();

/**
 * @brief Get and consume pending command
 * @param cmd Output command structure
 * @return true if command was available
 */
bool getCommand(Command& cmd);

/**
 * @brief Send responses
 */
void sendReady();
void sendBusy();
void sendDone();
void sendError(const char* msg = nullptr);

/**
 * @brief Send encoder values
 */
void sendEncoders(const int32_t values[NUM_ENCODERS]);

/**
 * @brief Send generic message
 */
void sendMessage(const char* msg);

} // namespace Serial_Cmd

#endif // APP_SERIAL_CMD_H
