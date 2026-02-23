/**
 * @file test/main.cpp
 * @brief Hardware Test Firmware - Motor & Encoder Pin Diagnostics
 *
 * Minimal firmware for testing individual PCA9685 channels, motors,
 * and encoders via serial commands. Allows direct pin manipulation
 * to verify wiring and direction mapping.
 *
 * Build: pio run -e hwtest
 * Monitor: pio device monitor -e hwtest
 *
 * Commands (send via serial 115200 baud):
 *   PIN,<ch>,<duty>    Set raw PCA9685 channel (0-15), duty 0-4095
 *   MOT,<idx>,<speed>  Set motor 0-3 (FL/RL/RR/FR), speed -255..255
 *   ENC                Read all encoder values
 *   ENCR               Reset all encoders to zero
 *   STOP               Coast all motors (freewheel)
 *   BRK                Brake all motors (short windings)
 *   MAP                Show hardware pin mapping
 *   I2C                Scan I2C bus for devices
 *   LIVE               Toggle live encoder streaming (10Hz)
 *   HELP               Show command list
 */

#include <Arduino.h>
#include <Wire.h>
#include "../hal/pca9685.h"
#include "../hal/motor.h"
#include "../hal/encoder.h"
#include "../core/config.h"

// ── State ───────────────────────────────────────────────────────────────────

static char cmdBuf[64];
static uint8_t cmdIdx = 0;
static bool liveMode = false;
static uint32_t lastLive = 0;

// Motor name lookup
static const char* const MOTOR_NAMES[] = {"FL", "RL", "RR", "FR"};

// Channel mapping for display
static const uint8_t MOTOR_CHANNELS[][2] = {
    {FL_IN1_CH, FL_IN2_CH},
    {RL_IN1_CH, RL_IN2_CH},
    {RR_IN1_CH, RR_IN2_CH},
    {FR_IN1_CH, FR_IN2_CH}
};

static const uint8_t ENC_PORTS[] = {FL_ENC_PORT, RL_ENC_PORT, RR_ENC_PORT, FR_ENC_PORT};
static const int8_t  ENC_DIRS[]  = {FL_ENC_DIR,  RL_ENC_DIR,  RR_ENC_DIR,  FR_ENC_DIR};

// ── Helpers ─────────────────────────────────────────────────────────────────

static void printSep() {
    Serial.println(F("--------------------------------"));
}

static int16_t parseIntArg(const char* str) {
    while (*str == ' ' || *str == ',') str++;
    return atoi(str);
}

static const char* nextArg(const char* str) {
    while (*str && *str != ',') str++;
    if (*str == ',') str++;
    return str;
}

// ── Command Handlers ────────────────────────────────────────────────────────

static void cmdHelp() {
    printSep();
    Serial.println(F("=== HW TEST COMMANDS ==="));
    Serial.println(F("PIN,<ch>,<duty>   Raw PCA9685 ch 0-15, duty 0-4095"));
    Serial.println(F("MOT,<idx>,<spd>   Motor 0-3, speed -255..255"));
    Serial.println(F("ENC               Read all encoders"));
    Serial.println(F("ENCR              Reset encoders"));
    Serial.println(F("STOP              Coast all motors"));
    Serial.println(F("BRK               Brake all motors"));
    Serial.println(F("MAP               Show pin mapping"));
    Serial.println(F("I2C               Scan I2C bus"));
    Serial.println(F("LIVE              Toggle encoder stream"));
    Serial.println(F("HELP              This help"));
    printSep();
}

static void cmdMap() {
    printSep();
    Serial.println(F("=== PIN MAPPING ==="));
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        Serial.print(F("Motor "));
        Serial.print(i);
        Serial.print(F(" ("));
        Serial.print(MOTOR_NAMES[i]);
        Serial.print(F("): IN1=CH"));
        Serial.print(MOTOR_CHANNELS[i][0]);
        Serial.print(F(" IN2=CH"));
        Serial.print(MOTOR_CHANNELS[i][1]);
        Serial.print(F(" | Enc port="));
        Serial.print(ENC_PORTS[i]);
        Serial.print(F(" dir="));
        Serial.println(ENC_DIRS[i]);
    }
    printSep();
}

static void cmdPin(const char* args) {
    int16_t ch = parseIntArg(args);
    args = nextArg(args);
    int16_t duty = parseIntArg(args);

    if (ch < 0 || ch > 15) {
        Serial.println(F("ERR: ch 0-15"));
        return;
    }
    if (duty < 0) duty = 0;
    if (duty > 4095) duty = 4095;

    PCA9685::setDuty((uint8_t)ch, (uint16_t)duty);

    Serial.print(F("PIN CH"));
    Serial.print(ch);
    Serial.print(F(" = "));
    Serial.println(duty);
}

static void cmdMotor(const char* args) {
    int16_t idx = parseIntArg(args);
    args = nextArg(args);
    int16_t speed = parseIntArg(args);

    if (idx < 0 || idx >= NUM_MOTORS) {
        Serial.println(F("ERR: motor 0-3 (FL=0,RL=1,RR=2,FR=3)"));
        return;
    }

    Motor::setDirect((uint8_t)idx, speed);

    Serial.print(F("MOT "));
    Serial.print(MOTOR_NAMES[idx]);
    Serial.print(F("["));
    Serial.print(idx);
    Serial.print(F("] = "));
    Serial.println(speed);
}

static void cmdEnc() {
    Encoder::Snapshot snap;
    Encoder::getSnapshot(snap);

    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
        Serial.print(MOTOR_NAMES[i]);
        Serial.print(F(":"));
        Serial.print(snap.ticks[i]);
        if (i < NUM_ENCODERS - 1) Serial.print(F(" | "));
    }
    Serial.println();
}

static void cmdI2C() {
    Serial.println(F("=== I2C SCAN ==="));
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("  0x"));
            if (addr < 16) Serial.print('0');
            Serial.print(addr, HEX);
            if (addr == PCA9685_I2C_ADDR) Serial.print(F(" (PCA9685)"));
            Serial.println();
            found++;
        }
    }
    Serial.print(found);
    Serial.println(F(" device(s) found"));
}

// ── Command Dispatcher ──────────────────────────────────────────────────────

static void processCommand(const char* cmd) {
    // Skip whitespace
    while (*cmd == ' ') cmd++;
    if (*cmd == '\0') return;

    if (strncmp(cmd, "PIN", 3) == 0) {
        cmdPin(cmd + 3);
    } else if (strncmp(cmd, "MOT", 3) == 0) {
        cmdMotor(cmd + 3);
    } else if (strcmp(cmd, "ENC") == 0) {
        cmdEnc();
    } else if (strcmp(cmd, "ENCR") == 0) {
        Encoder::resetAll();
        Serial.println(F("Encoders reset"));
    } else if (strcmp(cmd, "STOP") == 0) {
        Motor::coastAll();
        Serial.println(F("All motors coasting"));
    } else if (strcmp(cmd, "BRK") == 0) {
        Motor::brakeAll();
        Serial.println(F("All motors braking"));
    } else if (strcmp(cmd, "MAP") == 0) {
        cmdMap();
    } else if (strcmp(cmd, "I2C") == 0) {
        cmdI2C();
    } else if (strcmp(cmd, "LIVE") == 0) {
        liveMode = !liveMode;
        Serial.print(F("Live mode: "));
        Serial.println(liveMode ? F("ON (10Hz)") : F("OFF"));
    } else if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "?") == 0) {
        cmdHelp();
    } else {
        Serial.print(F("Unknown: "));
        Serial.println(cmd);
        Serial.println(F("Type HELP for commands"));
    }
}

// ── Arduino Entry Points ────────────────────────────────────────────────────

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {}

    Serial.println();
    printSep();
    Serial.println(F("  HW TEST FIRMWARE v1.0"));
    Serial.println(F("  Motor & Encoder Diagnostics"));
    printSep();

    Motor::init();
    Encoder::init();

    // Verify I2C
    if (Motor::isReady()) {
        Serial.println(F("PCA9685: OK"));
    } else {
        Serial.println(F("PCA9685: FAIL"));
    }

    // Quick I2C scan
    cmdI2C();

    Serial.println();
    Serial.println(F("Type HELP for commands"));
    Serial.print(F("> "));
}

void loop() {
    // Read serial commands
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdIdx > 0) {
                cmdBuf[cmdIdx] = '\0';

                // Convert to uppercase for case-insensitive matching
                for (uint8_t i = 0; i < cmdIdx; i++) {
                    if (cmdBuf[i] >= 'a' && cmdBuf[i] <= 'z') {
                        cmdBuf[i] -= 32;
                    }
                }

                processCommand(cmdBuf);
                cmdIdx = 0;
                Serial.print(F("> "));
            }
        } else if (cmdIdx < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdIdx++] = c;
        }
    }

    // Live encoder streaming at 10Hz
    if (liveMode) {
        uint32_t now = millis();
        if (now - lastLive >= 100) {
            lastLive = now;
            cmdEnc();
        }
    }
}
