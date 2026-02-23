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

#ifdef ENABLE_PS2
#include "PS2X_lib.h"
#endif
#include <Arduino.h>

//==============================================================================
// PS2 Controller (optional, enable with #define ENABLE_PS2 in config.h)
//==============================================================================

namespace {
#ifdef ENABLE_PS2
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

        if (ps2.ButtonDataByte() != 0xFFFF) {
            ps2LastSuccess = now;
        } else if (now - ps2LastSuccess > PS2_DISCONNECT_MS) {
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

        if (l2 || r2) {
            if (ps2.Button(PSB_PAD_UP))
                return l2 ? Direction::DIAG_FL : Direction::DIAG_FR;
            if (ps2.Button(PSB_PAD_DOWN))
                return l2 ? Direction::DIAG_BL : Direction::DIAG_BR;
        }

        if (ps2.Button(PSB_L1)) return Direction::LEFT;
        if (ps2.Button(PSB_R1)) return Direction::RIGHT;

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
#else
    constexpr bool ps2Connected = false;
    constexpr bool ps2Active = false;
#endif
}

//==============================================================================
// State Machine
//==============================================================================

namespace {
    State currentState = State::IDLE;
    uint32_t stateStartTime = 0;
    uint32_t lastCommandTime = 0;  // Watchdog: last time we received a command
    bool watchdogEnabled = true;   // Can be disabled for PS2 control

    // I2C health monitoring
    uint32_t lastHealthCheck = 0;
    uint8_t consecutiveI2CErrors = 0;

    const char* stateNames[] = {"IDLE", "MOVING", "CALIBRATING", "TESTING", "ERROR"};

    // ODOM tracking for VEL mode
    bool velocityActive = false;        // true when in VEL command mode
    int32_t odomPrevTicks[NUM_MOTORS];  // Previous encoder readings for delta

    void printOdom() {
        Encoder::Snapshot s;
        Encoder::getSnapshot(s);

        int16_t delta[NUM_MOTORS];
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            delta[i] = (int16_t)(s.ticks[i] - odomPrevTicks[i]);
            odomPrevTicks[i] = s.ticks[i];
        }

        int16_t vx, vy, wz;
        Mecanum::forwardKinematics(delta, vx, vy, wz);

        Serial.print(F("ODOM,"));
        Serial.print(vx);
        Serial.print(',');
        Serial.print(vy);
        Serial.print(',');
        Serial.println(wz);
    }

    // Diagnostic test state
    uint8_t testMotorIdx = 0;       // Which motor is being tested
    int16_t testMotorPWM = 0;       // Raw PWM applied
    bool testEncoderOnly = false;   // true = TENC (no motor), false = TMOTOR

    const char* motorName(uint8_t idx) {
        static const char* names[] = {"FL", "RL", "RR", "FR"};
        return (idx < NUM_MOTORS) ? names[idx] : "??";
    }

    void printEncoderTelemetry() {
        Encoder::Snapshot s;
        Encoder::getSnapshot(s);
        Serial.print(F("ENC,FL:"));
        Serial.print(s.ticks[MOTOR_FL]);
        Serial.print(F(",FR:"));
        Serial.print(s.ticks[MOTOR_FR]);
        Serial.print(F(",RL:"));
        Serial.print(s.ticks[MOTOR_RL]);
        Serial.print(F(",RR:"));
        Serial.print(s.ticks[MOTOR_RR]);
        Serial.print(F(",t_us:"));
        Serial.println(s.timestamp_us);
    }

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
                if (Motion::isCalibrating()) {
                    Motion::abortCalibration();
                }
                Motion::stop(true);
#ifdef ENABLE_PS2
                ps2Active = false;
#endif
                velocityActive = false;
                watchdogEnabled = true;
                setState(State::IDLE);
                Serial_Cmd::sendDone();
                return true;

            case CmdType::CALIBRATE:
                if (currentState != State::IDLE) {
                    Serial_Cmd::sendBusy();
                    return false;
                }
                Motion::startCalibration();
                watchdogEnabled = false;
                setState(State::CALIBRATING);
                return true;

            case CmdType::READ_ENCODERS: {
                Encoder::Snapshot snap;
                Encoder::getSnapshot(snap);
                Serial_Cmd::sendEncoders(snap.ticks);
                return true;
            }

            case CmdType::MOVE:
                Serial.print(F("MOVE cmd: dir="));
                Serial.print((int)cmd.dir);
                Serial.print(F(" spd="));
                Serial.print(cmd.speed);
                Serial.print(F(" ticks="));
                Serial.println(cmd.ticks);

                velocityActive = false;
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

            case CmdType::VELOCITY: {
                int16_t motorSpeeds[NUM_MOTORS];
                Mecanum::computeFromVelocity(cmd.vx, cmd.vy, cmd.wz, motorSpeeds);
                Motion::setMotorVelocities(motorSpeeds);
                watchdogEnabled = true;

                if (!velocityActive) {
                    // First VEL: init ODOM tracking and send ACK
                    velocityActive = true;
                    Encoder::Snapshot snap;
                    Encoder::getSnapshot(snap);
                    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                        odomPrevTicks[i] = snap.ticks[i];
                    }
                    Serial_Cmd::sendMessage("OK");
                }
                if (currentState != State::MOVING) {
                    setState(State::MOVING);
                }
                return true;
            }

            case CmdType::TEST_MOTOR:
                if (currentState != State::IDLE && currentState != State::TESTING) {
                    Serial_Cmd::sendBusy();
                    return false;
                }
                // Stop any previous test
                Motor::coastAll();
                Motion::clearTarget();
                Encoder::resetAll();

                testMotorIdx = cmd.motorIndex;
                testMotorPWM = cmd.pwm;
                testEncoderOnly = false;

                // Apply raw PWM (no gains, no PID)
                Motor::setDirect(testMotorIdx, testMotorPWM);

                Serial.print(F("TMOTOR,"));
                Serial.print(motorName(testMotorIdx));
                Serial.print(F(",pwm:"));
                Serial.println(testMotorPWM);

                setState(State::TESTING);
                return true;

            case CmdType::TEST_ENCODER:
                if (currentState != State::IDLE && currentState != State::TESTING) {
                    Serial_Cmd::sendBusy();
                    return false;
                }
                Motor::coastAll();
                Motion::clearTarget();
                Encoder::resetAll();

                testEncoderOnly = true;
                Serial.println(F("TENC started"));

                setState(State::TESTING);
                return true;

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
        if (millis() - lastEnc > IDLE_TELEMETRY_MS) {
            lastEnc = millis();
            printEncoderTelemetry();
        }
#ifdef ENABLE_PS2
        if (ps2Connected && ps2HasInput()) {
            Direction dir = ps2GetDir();
            int16_t speed = ps2GetSpeed();

            if (dir != Direction::STOP) {
                Motion::setVelocity(dir, speed);
                ps2Active = true;
                watchdogEnabled = false;
                setState(State::MOVING);
            }
        }
#endif
    }

    void handleMoving() {
        static uint32_t lastDebug = 0;
        static uint32_t lastEnc = 0;

        // 20Hz telemetry: ODOM in VEL mode, ENC in position mode
        if (millis() - lastEnc > MOVING_TELEMETRY_MS) {
            lastEnc = millis();
            if (velocityActive) {
                printOdom();
            } else {
                printEncoderTelemetry();
            }
        }

        // Periodic debug (position mode only, 1Hz)
        if (!velocityActive && millis() - lastDebug > MOVING_DEBUG_MS) {
            lastDebug = millis();
            Serial.print(F("Moving: remain="));
            Serial.println(Motion::remaining());
        }

#ifdef ENABLE_PS2
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
            watchdogEnabled = true;
            setState(State::IDLE);
            return;
        }
#endif

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
            Motor::startBrake(BRAKE_HOLD_MS);
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

    void handleTesting() {
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint < MOVING_TELEMETRY_MS) return;
        lastPrint = millis();

        Encoder::Snapshot snap;
        Encoder::getSnapshot(snap);

        if (testEncoderOnly) {
            // TENC mode: stream all encoders (user spins wheels manually)
            printEncoderTelemetry();
        } else {
            // TMOTOR mode: show all encoders so user can verify pairing
            // The tested motor's encoder should be moving, others near 0
            Serial.print(F("TEST,"));
            Serial.print(motorName(testMotorIdx));
            Serial.print(F(",pwm:"));
            Serial.print(testMotorPWM);
            Serial.print(F(",FL:"));
            Serial.print(snap.ticks[MOTOR_FL]);
            Serial.print(F(",FR:"));
            Serial.print(snap.ticks[MOTOR_FR]);
            Serial.print(F(",RL:"));
            Serial.print(snap.ticks[MOTOR_RL]);
            Serial.print(F(",RR:"));
            Serial.println(snap.ticks[MOTOR_RR]);
        }
    }

    void handleCalibrating() {
        // Calibration state machine is driven by onTick() at 50Hz.
        // Here we just check if it finished or timed out.
        if (!Motion::isCalibrating()) {
            Motor::startBrake(BRAKE_HOLD_MS);
            Serial_Cmd::sendDone();
            setState(State::IDLE);
            return;
        }
        // Safety timeout (3 sessions × (settle+measure+brake) + margin)
        if (millis() - stateStartTime > 30000UL) {
            Motion::abortCalibration();
            Serial_Cmd::sendError("Calib timeout");
            setState(State::IDLE);
        }
    }

    void handleError() {
        Motor::coastAll();
        if (millis() - stateStartTime > ERROR_RECOVERY_MS) {
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

    // Load per-motor calibration from EEPROM
    if (Motion::loadCalibFromEEPROM()) {
        const uint8_t* cal = Motion::getCalMaxTickrate();
        Serial.print(F("CALIB,loaded,FL:"));
        Serial.print(cal[MOTOR_FL]);
        Serial.print(F(",RL:"));
        Serial.print(cal[MOTOR_RL]);
        Serial.print(F(",RR:"));
        Serial.print(cal[MOTOR_RR]);
        Serial.print(F(",FR:"));
        Serial.println(cal[MOTOR_FR]);
    } else {
        Serial.println(F("No calibration in EEPROM, using defaults"));
    }

#ifdef ENABLE_PS2
    Serial.println(F("PS2 init..."));
    ps2Init();
#endif

    currentState = State::IDLE;
    stateStartTime = millis();
    lastCommandTime = millis();
    velocityActive = false;

    Serial.println(F("Init complete"));
    Serial_Cmd::sendReady();
    Serial_Cmd::sendMessage("Robot initialized");
}

void update() {
    // Service non-blocking brake timer
    Motor::updateBrake();

    // Periodic I2C health check
    if (millis() - lastHealthCheck >= I2C_HEALTH_CHECK_MS) {
        lastHealthCheck = millis();
        if (Motor::checkHealth()) {
            consecutiveI2CErrors = 0;
        } else {
            consecutiveI2CErrors++;
            if (consecutiveI2CErrors >= I2C_ERROR_THRESHOLD) {
                Motor::coastAll();
                Motion::clearTarget();
                Serial_Cmd::sendError("I2C fault");
                setState(State::ERROR);
                consecutiveI2CErrors = 0;
            }
        }
    }

#ifdef ENABLE_PS2
    ps2Update();

    static uint32_t lastReconnect = 0;
    if (!ps2Connected && (millis() - lastReconnect > PS2_RECONNECT_MS)) {
        lastReconnect = millis();
        ps2Init();
    }
#endif

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
            handleCalibrating();
            break;
        case State::TESTING:
            handleTesting();
            break;
        case State::ERROR:
            handleError();
            break;
    }
}

void onTick() {
    if (Motion::isCalibrating()) {
        Motion::updateCalibration();
    } else {
        Motion::update();
    }
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
#ifdef ENABLE_PS2
    ps2Active = false;
#endif
    velocityActive = false;
    setState(State::IDLE);
}

uint32_t stateTime() {
    return millis() - stateStartTime;
}

bool isPS2Active() {
#ifdef ENABLE_PS2
    return ps2Active;
#else
    return false;
#endif
}

} // namespace Robot
