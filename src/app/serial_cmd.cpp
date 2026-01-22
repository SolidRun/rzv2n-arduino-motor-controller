/**
 * @file serial_cmd.cpp
 * @brief Serial command parser implementation
 *
 * Character-based buffer parsing (no String class).
 * Non-blocking - call update() each loop iteration.
 */

#include "serial_cmd.h"
#include <Arduino.h>

//==============================================================================
// Private State
//==============================================================================

namespace {
    char buffer[CMD_BUFFER_SIZE];
    uint8_t bufferIdx = 0;
    bool cmdReady = false;
    Command pendingCmd;
}

//==============================================================================
// Parsing Helpers
//==============================================================================

namespace {
    // Compare command prefix
    bool match(const char* input, const char* cmd) {
        while (*cmd) {
            if (*input != *cmd) return false;
            input++;
            cmd++;
        }
        return (*input == '\0' || *input == ',');
    }

    // Parse integer from string
    int32_t parseNum(const char* str) {
        int32_t result = 0;
        bool neg = false;

        if (*str == '-') {
            neg = true;
            str++;
        }

        while (*str >= '0' && *str <= '9') {
            result = result * 10 + (*str - '0');
            str++;
        }

        return neg ? -result : result;
    }

    // Find comma or end
    const char* toComma(const char* s) {
        while (*s && *s != ',') s++;
        return s;
    }

    // Skip comma
    const char* afterComma(const char* s) {
        if (*s == ',') s++;
        return s;
    }

    // Parse command from buffer
    void parse() {
        pendingCmd = {CmdType::NONE, Direction::STOP, 0, 0, 0, 0};

        // Simple commands
        if (match(buffer, "STOP")) {
            pendingCmd.type = CmdType::STOP;
            cmdReady = true;
            return;
        }

        if (match(buffer, "READ")) {
            pendingCmd.type = CmdType::READ_ENCODERS;
            cmdReady = true;
            return;
        }

        if (match(buffer, "SYNC")) {
            pendingCmd.type = CmdType::CALIBRATE;
            cmdReady = true;
            return;
        }
        // TWIST,v_mmps,w_mradps
        if (match(buffer, "TWIST")) {
            const char* p = afterComma(toComma(buffer));
            int32_t v_mmps = parseNum(p);

            p = afterComma(toComma(p));
            int32_t w_mradps = parseNum(p);

            pendingCmd.type = CmdType::TWIST;
            pendingCmd.v_mmps = v_mmps;
            pendingCmd.w_mradps = w_mradps;
            cmdReady = true;
            return;
        }       

        // FWD,speed,ticks
        if (match(buffer, "FWD")) {
            const char* p = afterComma(toComma(buffer));
            int16_t speed = (int16_t)parseNum(p);

            p = afterComma(toComma(p));
            int32_t ticks = parseNum(p);

            if (speed > 0 && ticks > 0) {
                pendingCmd.type = CmdType::MOVE;
                pendingCmd.dir = Direction::FORWARD;
                pendingCmd.speed = speed;
                pendingCmd.ticks = ticks;
                cmdReady = true;
                return;
            }
        }

        // BWD,speed,ticks
        if (match(buffer, "BWD")) {
            const char* p = afterComma(toComma(buffer));
            int16_t speed = (int16_t)parseNum(p);

            p = afterComma(toComma(p));
            int32_t ticks = parseNum(p);

            if (speed > 0 && ticks > 0) {
                pendingCmd.type = CmdType::MOVE;
                pendingCmd.dir = Direction::BACKWARD;
                pendingCmd.speed = speed;
                pendingCmd.ticks = ticks;
                cmdReady = true;
                return;
            }
        }

        // TURN,speed,ticks (positive=left, negative=right)
        if (match(buffer, "TURN")) {
            const char* p = afterComma(toComma(buffer));
            int16_t speed = (int16_t)parseNum(p);

            p = afterComma(toComma(p));
            int32_t rawTicks = parseNum(p);

            if (speed > 0) {
                pendingCmd.type = CmdType::MOVE;
                if (rawTicks >= 0) {
                    pendingCmd.dir = Direction::ROTATE_CCW;
                    pendingCmd.ticks = rawTicks;
                } else {
                    pendingCmd.dir = Direction::ROTATE_CW;
                    pendingCmd.ticks = -rawTicks;
                }
                pendingCmd.speed = speed;
                cmdReady = true;
                return;
            }
        }

        // LEFT,speed,ticks (strafe)
        if (match(buffer, "LEFT")) {
            const char* p = afterComma(toComma(buffer));
            int16_t speed = (int16_t)parseNum(p);

            p = afterComma(toComma(p));
            int32_t ticks = parseNum(p);

            if (speed > 0 && ticks > 0) {
                pendingCmd.type = CmdType::MOVE;
                pendingCmd.dir = Direction::LEFT;
                pendingCmd.speed = speed;
                pendingCmd.ticks = ticks;
                cmdReady = true;
                return;
            }
        }

        // RIGHT,speed,ticks (strafe)
        if (match(buffer, "RIGHT")) {
            const char* p = afterComma(toComma(buffer));
            int16_t speed = (int16_t)parseNum(p);

            p = afterComma(toComma(p));
            int32_t ticks = parseNum(p);

            if (speed > 0 && ticks > 0) {
                pendingCmd.type = CmdType::MOVE;
                pendingCmd.dir = Direction::RIGHT;
                pendingCmd.speed = speed;
                pendingCmd.ticks = ticks;
                cmdReady = true;
                return;
            }
        }

        // DIAGFL,speed,ticks (diagonal forward-left)
        if (match(buffer, "DIAGFL")) {
            const char* p = afterComma(toComma(buffer));
            int16_t speed = (int16_t)parseNum(p);

            p = afterComma(toComma(p));
            int32_t ticks = parseNum(p);

            if (speed > 0 && ticks > 0) {
                pendingCmd.type = CmdType::MOVE;
                pendingCmd.dir = Direction::DIAG_FL;
                pendingCmd.speed = speed;
                pendingCmd.ticks = ticks;
                cmdReady = true;
                return;
            }
        }

        // DIAGFR,speed,ticks (diagonal forward-right)
        if (match(buffer, "DIAGFR")) {
            const char* p = afterComma(toComma(buffer));
            int16_t speed = (int16_t)parseNum(p);

            p = afterComma(toComma(p));
            int32_t ticks = parseNum(p);

            if (speed > 0 && ticks > 0) {
                pendingCmd.type = CmdType::MOVE;
                pendingCmd.dir = Direction::DIAG_FR;
                pendingCmd.speed = speed;
                pendingCmd.ticks = ticks;
                cmdReady = true;
                return;
            }
        }

        // DIAGBL,speed,ticks (diagonal backward-left)
        if (match(buffer, "DIAGBL")) {
            const char* p = afterComma(toComma(buffer));
            int16_t speed = (int16_t)parseNum(p);

            p = afterComma(toComma(p));
            int32_t ticks = parseNum(p);

            if (speed > 0 && ticks > 0) {
                pendingCmd.type = CmdType::MOVE;
                pendingCmd.dir = Direction::DIAG_BL;
                pendingCmd.speed = speed;
                pendingCmd.ticks = ticks;
                cmdReady = true;
                return;
            }
        }

        // DIAGBR,speed,ticks (diagonal backward-right)
        if (match(buffer, "DIAGBR")) {
            const char* p = afterComma(toComma(buffer));
            int16_t speed = (int16_t)parseNum(p);

            p = afterComma(toComma(p));
            int32_t ticks = parseNum(p);

            if (speed > 0 && ticks > 0) {
                pendingCmd.type = CmdType::MOVE;
                pendingCmd.dir = Direction::DIAG_BR;
                pendingCmd.speed = speed;
                pendingCmd.ticks = ticks;
                cmdReady = true;
                return;
            }
        }

        // Unknown command
        pendingCmd.type = CmdType::UNKNOWN;
        cmdReady = true;
    }
}

//==============================================================================
// Public Interface
//==============================================================================

namespace Serial_Cmd {

void init() {
    Serial.begin(SERIAL_BAUD);
    // Wait for Serial to be ready (important for some Arduino boards)
    while (!Serial) {
        ; // Wait for serial port to connect (needed for native USB)
    }
    delay(100);  // Small delay for serial stability
    bufferIdx = 0;
    cmdReady = false;
    buffer[0] = '\0';
}

void update() {
    static bool overflow = false;  // Track if we're in overflow state

    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (overflow) {
                // Previous command overflowed - send error and reset
                sendError("Buffer overflow");
                overflow = false;
            } else if (bufferIdx > 0) {
                buffer[bufferIdx] = '\0';
                parse();
            }
            bufferIdx = 0;
        } else if (bufferIdx < CMD_BUFFER_SIZE - 1) {
            buffer[bufferIdx++] = c;
        } else {
            // Buffer overflow - set flag to notify on newline
            overflow = true;
        }
    }
}

bool hasCommand() {
    return cmdReady;
}

bool getCommand(Command& cmd) {
    if (!cmdReady) return false;

    cmd = pendingCmd;
    cmdReady = false;
    pendingCmd.type = CmdType::NONE;

    return true;
}

void sendReady() {
    Serial.println(F("READY"));
}

void sendBusy() {
    Serial.println(F("BUSY"));
}

void sendDone() {
    Serial.println(F("DONE"));
}

void sendError(const char* msg) {
    Serial.print(F("ERROR"));
    if (msg) {
        Serial.print(F(": "));
        Serial.println(msg);
    } else {
        Serial.println();
    }
}

void sendEncoders(const int32_t values[NUM_ENCODERS]) {
    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
        Serial.println(values[i]);
    }
}

void sendGains(const float gains[NUM_MOTORS]) {
    Serial.print(F("Gains: "));
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        Serial.print(gains[i], 3);
        if (i < NUM_MOTORS - 1) Serial.print(F(", "));
    }
    Serial.println();
}

void sendMessage(const char* msg) {
    Serial.println(msg);
}

} // namespace Serial_Cmd
