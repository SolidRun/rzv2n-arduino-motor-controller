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

    // Movement command lookup table: CMD,speed,ticks format
    // To add a new direction: just add one line to this table
    struct MoveCmd {
        const char* name;
        Direction dir;
    };

    const MoveCmd moveCmds[] = {
        {"FWD",    Direction::FORWARD},
        {"BWD",    Direction::BACKWARD},
        {"LEFT",   Direction::LEFT},
        {"RIGHT",  Direction::RIGHT},
        {"DIAGFL", Direction::DIAG_FL},
        {"DIAGFR", Direction::DIAG_FR},
        {"DIAGBL", Direction::DIAG_BL},
        {"DIAGBR", Direction::DIAG_BR},
    };
    constexpr uint8_t NUM_MOVE_CMDS = sizeof(moveCmds) / sizeof(moveCmds[0]);

    // Try to parse a standard move command (CMD,speed,ticks)
    bool tryParseMove(const char* buf) {
        for (uint8_t i = 0; i < NUM_MOVE_CMDS; i++) {
            if (match(buf, moveCmds[i].name)) {
                const char* p = afterComma(toComma(buf));
                int16_t speed = (int16_t)parseNum(p);
                p = afterComma(toComma(p));
                int32_t ticks = parseNum(p);

                if (speed > 0 && ticks > 0) {
                    pendingCmd.type = CmdType::MOVE;
                    pendingCmd.dir = moveCmds[i].dir;
                    pendingCmd.speed = speed;
                    pendingCmd.ticks = ticks;
                    cmdReady = true;
                    return true;
                }
                return false;  // Matched name but invalid params
            }
        }
        return false;  // No match
    }

    // Parse motor index from name (FL/FR/RL/RR) or number (0-3)
    int8_t parseMotorIndex(const char* s) {
        if (s[0] == 'F' && s[1] == 'L') return MOTOR_FL;
        if (s[0] == 'F' && s[1] == 'R') return MOTOR_FR;
        if (s[0] == 'R' && s[1] == 'L') return MOTOR_RL;
        if (s[0] == 'R' && s[1] == 'R') return MOTOR_RR;
        int32_t idx = parseNum(s);
        if (idx >= 0 && idx < NUM_MOTORS) return (int8_t)idx;
        return -1;
    }

    // Parse command from buffer
    void parse() {
        pendingCmd = {CmdType::NONE, Direction::STOP, 0, 0, 0, 0, 0, 0, 0};

        // Simple commands (no parameters)
        if (match(buffer, "STOP"))  { pendingCmd.type = CmdType::STOP;          cmdReady = true; return; }
        if (match(buffer, "READ"))  { pendingCmd.type = CmdType::READ_ENCODERS; cmdReady = true; return; }
        if (match(buffer, "TENC"))  { pendingCmd.type = CmdType::TEST_ENCODER;  cmdReady = true; return; }
        if (match(buffer, "CALIB")) { pendingCmd.type = CmdType::CALIBRATE;    cmdReady = true; return; }

        // TURN has special sign-based direction logic (positive=CCW, negative=CW)
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

        // TMOTOR,index,pwm - direct single motor test
        // index: 0-3 or FL/FR/RL/RR, pwm: -255 to 255
        if (match(buffer, "TMOTOR")) {
            const char* p = afterComma(toComma(buffer));
            int8_t idx = parseMotorIndex(p);
            p = afterComma(toComma(p));
            int16_t pwm = (int16_t)parseNum(p);

            if (idx >= 0) {
                pendingCmd.type = CmdType::TEST_MOTOR;
                pendingCmd.motorIndex = (uint8_t)idx;
                pendingCmd.pwm = constrain(pwm, (int16_t)-255, (int16_t)255);
                cmdReady = true;
                return;
            }
        }

        // VEL,vx,vy,wz - continuous velocity control
        if (match(buffer, "VEL")) {
            const char* p = afterComma(toComma(buffer));
            int16_t vx = (int16_t)parseNum(p);
            p = afterComma(toComma(p));
            int16_t vy = (int16_t)parseNum(p);
            p = afterComma(toComma(p));
            int16_t wz = (int16_t)parseNum(p);

            pendingCmd.type = CmdType::VELOCITY;
            pendingCmd.vx = vx;
            pendingCmd.vy = vy;
            pendingCmd.wz = wz;
            cmdReady = true;
            return;
        }

        // All standard directional moves (CMD,speed,ticks)
        if (tryParseMove(buffer)) return;

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

void sendMessage(const char* msg) {
    Serial.println(msg);
}

} // namespace Serial_Cmd
