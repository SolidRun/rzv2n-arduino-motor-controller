/**
 * @file robot.cpp
 * @brief Robot state machine implementation
 *
 * Coordinates all subsystems:
 *   - Serial command processing
 *   - PS2 controller (optional)
 *   - Motion control
 *   - State transitions
 */

#include "robot.h"
#include "motion.h"
#include "serial_cmd.h"
#include "mecanum.h"
#include "../hal/motor.h"
#include "../hal/encoder.h"
#include "../hal/timer.h"

// PS2 is optional - use weak linking
#include "PS2X_lib.h"
#include <Arduino.h>

//==============================================================================
// PS2 Controller
//==============================================================================

namespace {
    PS2X ps2;
    bool ps2Connected = false;
    bool ps2Active = false;
    uint32_t ps2LastRead = 0;
    uint32_t ps2LastSuccess = 0;

    bool ps2Init() {
        uint8_t error = ps2.config_gamepad(
            PS2_CLK_PIN, PS2_CMD_PIN, PS2_ATT_PIN, PS2_DAT_PIN,
            PS2_PRESSURES, PS2_RUMBLE);

        ps2Connected = (error == 0);
        if (ps2Connected) {
            ps2LastSuccess = millis();
        }
        return ps2Connected;
    }

    void ps2Update() {
        if (!ps2Connected) return;

        uint32_t now = millis();
        if (now - ps2LastRead < PS2_READ_MS) return;

        ps2LastRead = now;
        ps2.read_gamepad(false, 0);

        // Check for disconnect
        if (ps2.ButtonDataByte() != 0xFFFF) {
            ps2LastSuccess = now;
        } else if (now - ps2LastSuccess > 1000) {
            ps2Connected = false;
        }
    }

    bool ps2HasInput() {
        if (!ps2Connected) return false;

        return ps2.Button(PSB_PAD_UP) || ps2.Button(PSB_PAD_DOWN) ||
               ps2.Button(PSB_PAD_LEFT) || ps2.Button(PSB_PAD_RIGHT) ||
               ps2.Button(PSB_L1) || ps2.Button(PSB_R1) ||
               ps2.Button(PSB_L2) || ps2.Button(PSB_R2);
    }

    Direction ps2GetDir() {
        if (!ps2Connected) return Direction::STOP;

        bool l2 = ps2.Button(PSB_L2);
        bool r2 = ps2.Button(PSB_R2);

        // Diagonal with L2/R2
        if (l2 || r2) {
            if (ps2.Button(PSB_PAD_UP))
                return l2 ? Direction::DIAG_FL : Direction::DIAG_FR;
            if (ps2.Button(PSB_PAD_DOWN))
                return l2 ? Direction::DIAG_BL : Direction::DIAG_BR;
        }

        // Strafe with L1/R1
        if (ps2.Button(PSB_L1)) return Direction::LEFT;
        if (ps2.Button(PSB_R1)) return Direction::RIGHT;

        // Basic D-pad
        if (ps2.Button(PSB_PAD_UP)) return Direction::FORWARD;
        if (ps2.Button(PSB_PAD_DOWN)) return Direction::BACKWARD;
        if (ps2.Button(PSB_PAD_LEFT)) return Direction::ROTATE_CCW;
        if (ps2.Button(PSB_PAD_RIGHT)) return Direction::ROTATE_CW;

        return Direction::STOP;
    }

    int16_t ps2GetSpeed() {
        if (!ps2Connected) return 0;

        if (ps2.Button(PSB_TRIANGLE)) return PS2_SPEED_FAST;
        if (ps2.Button(PSB_CROSS)) return PS2_SPEED_SLOW;
        return PS2_SPEED_NORMAL;
    }
}

//==============================================================================
// State Machine
//==============================================================================

namespace {
    State currentState = State::IDLE;
    uint32_t stateStartTime = 0;
    uint32_t lastCommandTime = 0;  // Watchdog: last time we received a command
    bool watchdogEnabled = true;   // Can be disabled for PS2 control

    const char* stateNames[] = {"IDLE", "MOVING", "CALIBRATING", "ERROR"};

    void setState(State s) {
        if (currentState != s) {
            currentState = s;
            stateStartTime = millis();
            DBG_PRINT(F("State: "));
            DBG_PRINTLN(stateNames[(uint8_t)s]);
        }
    }

    // Handle commands from serial
    bool handleCommand(const Command& cmd) {
        // Feed the watchdog on any valid command
        lastCommandTime = millis();

        switch (cmd.type) {
            case CmdType::STOP:
                Motion::stop(true);
                ps2Active = false;
                watchdogEnabled = true;  // Re-enable watchdog after stop
                setState(State::IDLE);
                Serial_Cmd::sendDone();
                return true;

            case CmdType::READ_ENCODERS: {
                Encoder::Snapshot snap;
                Encoder::getSnapshot(snap);
                Serial_Cmd::sendEncoders(snap.ticks);
                return true;
            }

            case CmdType::CALIBRATE:
                if (currentState != State::IDLE) {
                    Serial_Cmd::sendBusy();
                    return false;
                }
                setState(State::CALIBRATING);
                if (Motion::calibrate()) {
                    // Print the calculated gains
                    float gains[NUM_MOTORS];
                    Motor::getAllGains(gains);
                    Serial_Cmd::sendGains(gains);
                    Serial_Cmd::sendDone();
                } else {
                    Serial_Cmd::sendError("Calibration failed");
                }
                setState(State::IDLE);
                return true;

            case CmdType::MOVE:
                Serial.print(F("MOVE cmd: dir="));
                Serial.print((int)cmd.dir);
                Serial.print(F(" spd="));
                Serial.print(cmd.speed);
                Serial.print(F(" ticks="));
                Serial.println(cmd.ticks);

                // Allow preempting current motion with new command
                if (currentState == State::MOVING && !ps2Active) {
                    // Stop current motion immediately, then start new one
                    Motor::coastAll();
                    Motion::clearTarget();
                    // Send DONE for the preempted command so client isn't left waiting
                    Serial_Cmd::sendDone();
                }
                else if (currentState != State::IDLE) {
                    // Only reject if calibrating or in error state
                    Serial_Cmd::sendBusy();
                    return false;
                }
                Motion::setTarget(cmd.dir, cmd.speed, cmd.ticks);
                // Disable watchdog for tick-based movements (they have MOVE_TIMEOUT)
                // Watchdog is only for velocity mode (continuous streaming)
                watchdogEnabled = false;
                setState(State::MOVING);
                // Send ACK so client knows command was accepted
                Serial_Cmd::sendMessage("OK");
                return true;    

            case CmdType::TWIST: {
                ps2Active = false;
                watchdogEnabled = false;

                Motion::setTwist(cmd.v_mmps, cmd.w_mradps);

                if (cmd.v_mmps == 0 && cmd.w_mradps == 0) {
                    setState(State::IDLE);
                } else {
                    setState(State::MOVING);
                }

                Serial_Cmd::sendMessage("OK");
                return true;
            }

            case CmdType::UNKNOWN:
                Serial_Cmd::sendError("Unknown");
                return false;

            default:
                return false;
        }
    }

    // State handlers
    void handleIdle() {
        static uint32_t lastEnc = 0;
        if (millis() - lastEnc > 1000) {  
            lastEnc = millis();

            Encoder::Snapshot s;
            Encoder::getSnapshot(s);

            Serial.print(F("ENC,FL:"));
            Serial.print(s.ticks[0]);
            Serial.print(F(",FR:"));
            Serial.print(s.ticks[3]);
            Serial.print(F(",RL:"));
            Serial.print(s.ticks[1]);
            Serial.print(F(",RR:"));
            Serial.print(s.ticks[2]);
            Serial.print(F(",t_us:"));
            Serial.println(s.timestamp_us);
        }
        // Check PS2 input
        if (ps2Connected && ps2HasInput()) {
            Direction dir = ps2GetDir();
            int16_t speed = ps2GetSpeed();

            if (dir != Direction::STOP) {
                Motion::setVelocity(dir, speed);
                ps2Active = true;
                watchdogEnabled = false;  // Disable watchdog for PS2 control
                setState(State::MOVING);
            }
        }
    }

    void handleMoving() {
        // Periodic debug output (every ~1 second)
        static uint32_t lastDebug = 0;
        static uint32_t lastEnc = 0;
        if (millis() - lastDebug > 1000) {
            lastDebug = millis();
            Serial.print(F("Moving: stalled="));
            Serial.print(Motion::isStalled());
            Serial.print(F(" complete="));
            Serial.print(Motion::isComplete());
            Serial.print(F(" remain="));
            Serial.println(Motion::remaining());
        }

        // 20Hz encoder stream
        if (millis() - lastEnc > 50) {  
            lastEnc = millis();

            Encoder::Snapshot s;
            Encoder::getSnapshot(s);

            Serial.print(F("ENC,FL:"));
            Serial.print(s.ticks[0]);
            Serial.print(F(",FR:"));
            Serial.print(s.ticks[3]);
            Serial.print(F(",RL:"));
            Serial.print(s.ticks[1]);
            Serial.print(F(",RR:"));
            Serial.print(s.ticks[2]);
            Serial.print(F(",t_us:"));

            Serial.println(s.timestamp_us);
        }

        // PS2 control
        if (ps2Active) {
            if (ps2Connected && ps2HasInput()) {
                Direction dir = ps2GetDir();
                int16_t speed = ps2GetSpeed();

                if (dir != Direction::STOP) {
                    Motion::setVelocity(dir, speed);
                    return;
                }
            }

            // PS2 released
            Motion::stop(false);
            ps2Active = false;
            watchdogEnabled = true;  // Re-enable watchdog
            setState(State::IDLE);
            return;
        }

        if (!Motion::isMoving()) {
            Motor::coastAll();
            watchdogEnabled = true;
            setState(State::IDLE);
            return;
        }
        // Serial command - check for stall (encoder not making progress)
        if (Motion::isStalled()) {
            Motor::coastAll();
            Motion::clearTarget();
            watchdogEnabled = true;
            Serial_Cmd::sendError("Stalled");
            setState(State::IDLE);
            return;
        }

        // Serial command - check completion
        if (Motion::isComplete()) {
            Motor::brakeAndRelease(BRAKE_HOLD_MS);
            Motion::clearTarget();
            watchdogEnabled = true;
            Serial_Cmd::sendDone();
            setState(State::IDLE);
            return;
        }

        // Watchdog check - stop if no command received for too long
        if (watchdogEnabled && (millis() - lastCommandTime > WATCHDOG_TIMEOUT_MS)) {
            Motor::coastAll();
            Motion::clearTarget();
            watchdogEnabled = true;
            Serial_Cmd::sendError("Watchdog");
            setState(State::IDLE);
            return;
        }

        // Timeout check (for very long movements)
        if (millis() - stateStartTime > MOVE_TIMEOUT_MS) {
            Motor::coastAll();
            Motion::clearTarget();
            watchdogEnabled = true;
            Serial_Cmd::sendError("Timeout");
            setState(State::IDLE);
        }
    }

    void handleError() {
        Motor::coastAll();
        if (millis() - stateStartTime > 5000) {
            setState(State::IDLE);
        }
    }
}

//==============================================================================
// Public Interface
//==============================================================================

namespace Robot {

void init() {
    // Initialize Serial FIRST so we can see debug output
    Serial_Cmd::init();
    Serial.println(F("Starting init..."));

    // Initialize HAL
    Serial.println(F("Motor init..."));
    Motor::init();
    Serial.println(F("Encoder init..."));
    Encoder::init();
    Serial.println(F("Timer init..."));
    Timer::init();

    // Initialize app layers
    Serial.println(F("Motion init..."));
    Motion::init();

    // Try PS2 (non-blocking, optional)
    Serial.println(F("PS2 init..."));
    ps2Init();

    currentState = State::IDLE;
    stateStartTime = millis();
    lastCommandTime = millis();  // Initialize watchdog
    ps2Active = false;

    Serial.println(F("Init complete"));
    Serial_Cmd::sendReady();
    Serial_Cmd::sendMessage("Robot initialized");
}

void update() {
    // Update PS2
    ps2Update();

    // Try reconnect if disconnected
    static uint32_t lastReconnect = 0;
    if (!ps2Connected && (millis() - lastReconnect > PS2_RECONNECT_MS)) {
        lastReconnect = millis();
        ps2Init();
    }

    // Process serial commands
    Serial_Cmd::update();
    if (Serial_Cmd::hasCommand()) {
        Command cmd;
        if (Serial_Cmd::getCommand(cmd)) {
            handleCommand(cmd);
        }
    }

    // State machine
    switch (currentState) {
        case State::IDLE:
            handleIdle();
            break;
        case State::MOVING:
            handleMoving();
            break;
        case State::CALIBRATING:
            // Calibration is blocking, handled in command
            break;
        case State::ERROR:
            handleError();
            break;
    }
}

void onTick() {
    Motion::update();
}

State getState() {
    return currentState;
}

const char* getStateName() {
    return stateNames[(uint8_t)currentState];
}

void emergencyStop() {
    Motor::emergencyStop();
    Motion::clearTarget();
    ps2Active = false;
    setState(State::IDLE);
}

uint32_t stateTime() {
    return millis() - stateStartTime;
}

bool isPS2Active() {
    return ps2Active;
}

} // namespace Robot
