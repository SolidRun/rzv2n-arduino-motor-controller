/**
 * @file serial_cmd.h
 * @brief Serial command interface
 *
 * Non-blocking command parser for robot control.
 * Protocol: CMD,param1,param2\n
 *
 * Commands:
 *   STOP        - Emergency stop
 *   READ        - Read encoder values
 *   SYNC        - Run calibration
 *   FWDS        - Smart forward (default distance)
 *   FWD,s,t     - Forward: speed s, ticks t
 *   BWD,s,t     - Backward: speed s, ticks t
 *   TURN,s,t    - Turn: speed s, ticks t (positive=left, negative=right)
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
 * @brief Send motor gains
 */
void sendGains(const float gains[NUM_MOTORS]);

/**
 * @brief Send generic message
 */
void sendMessage(const char* msg);

} // namespace Serial_Cmd

#endif // APP_SERIAL_CMD_H
