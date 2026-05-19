#include <Wire.h>
#include <Motoron.h>
#include <MiniMessenger.h>
#include "secrets.h"

/*
 * GIGA Motor Safety Test
 *
 * This sketch integrates:
 * - Server heartbeat / emergency stop through MiniMessenger
 * - Mechanical kill switch
 * - Two DC motors controlled through Motoron I2C
 *
 * Motors are allowed to move only when:
 * 1. Server heartbeat enables the robot
 * 2. Mechanical kill switch is not pressed
 * 3. Heartbeat has not timed out
 */

MiniMessenger messenger;
MotoronI2C mc(16);

// --- MOTOR CONFIGURATION ---
const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;

const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;

const int MOTOR_SPEED = 300;

// --- MECHANICAL KILL SWITCH ---
const int KILL_SWITCH_PIN = 8;

/*
 * Wiring assumption:
 * One side of the switch -> GND
 * Other side -> D8
 *
 * pinMode uses INPUT_PULLUP.
 * Not pressed = HIGH
 * Pressed     = LOW
 */
const int KILL_SWITCH_PRESSED = LOW;

// --- MINI MESSENGER CONFIGURATION ---
const char* BoardId = "Wilson";

// --- SAFETY STATE ---
bool serverEnabled = false;

unsigned long lastRegisterMs = 0;
unsigned long lastHeartbeatMs = 0;

const unsigned long HEARTBEAT_TIMEOUT_MS = 1000;

// --- DEMO MOVEMENT STATE ---
unsigned long lastDirectionChangeMs = 0;
bool movingForward = true;


// Stop both motors immediately
void stopMotors() {
    mc.setSpeed(LEFT_MOTOR_CHANNEL, 0);
    mc.setSpeed(RIGHT_MOTOR_CHANNEL, 0);
}


// Read mechanical kill switch
bool isKillSwitchPressed() {
    return digitalRead(KILL_SWITCH_PIN) == KILL_SWITCH_PRESSED;
}


// Overall safety condition
bool isRobotAllowedToMove() {
    bool heartbeatOk = millis() - lastHeartbeatMs <= HEARTBEAT_TIMEOUT_MS;
    bool killSwitchOk = !isKillSwitchPressed();

    return serverEnabled && heartbeatOk && killSwitchOk;
}


// Runs every time an MQTT message arrives for this robot
void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
    char msg[128];

    size_t copyLen = (length < 127) ? length : 127;
    memcpy(msg, payload, copyLen);
    msg[copyLen] = '\0';

    // 1. Heartbeat message from server
    if (strstr(msg, "type=heartbeat")) {
        lastHeartbeatMs = millis();

        if (strstr(msg, "enable=1")) {
            serverEnabled = true;
        } else if (strstr(msg, "enable=0")) {
            serverEnabled = false;
            stopMotors();
            Serial.println("SAFETY: Server heartbeat disabled");
        }
    }

    // 2. Emergency stop or individual disable
    if (strstr(msg, "type=emergency enabled=true") ||
        strstr(msg, "type=disable enabled=false")) {
        serverEnabled = false;
        stopMotors();
        Serial.println("SAFETY: Emergency or disable command active");
    }
}


void setup() {
    Serial.begin(115200);

    pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);

    Serial.println("\n--- GIGA MOTOR SAFETY TEST STARTING ---");

    // --- Motoron setup ---
    Wire.begin();

    mc.reinitialize();
    mc.disableCrc();
    mc.clearResetFlag();

    mc.setMaxAcceleration(LEFT_MOTOR_CHANNEL, 300);
    mc.setMaxDeceleration(LEFT_MOTOR_CHANNEL, 300);
    mc.setMaxAcceleration(RIGHT_MOTOR_CHANNEL, 300);
    mc.setMaxDeceleration(RIGHT_MOTOR_CHANNEL, 300);

    stopMotors();

    // --- MiniMessenger setup ---
    messenger.onMessage(onMessage);
    messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);

    Serial.println("Network connecting...");
}


void loop() {
    // Required for receiving server messages
    messenger.loop();

    // --- Registration every 10 seconds ---
    if (millis() - lastRegisterMs > 10000 || lastRegisterMs == 0) {
        lastRegisterMs = millis();

        char reg[64];
        snprintf(reg,
                 sizeof(reg),
                 "type=register team_id=%s board_id=%s",
                 GROUP_ID,
                 BoardId);

        messenger.sendToBoard("server", reg);
        Serial.println("Registered with server.");
    }

    // --- Safety checks ---
    if (isKillSwitchPressed()) {
        stopMotors();
        Serial.println("SAFETY: Mechanical kill switch pressed");
        delay(100);
        return;
    }

    if (serverEnabled && millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
        serverEnabled = false;
        stopMotors();
        Serial.println("SAFETY: Heartbeat timeout");
        delay(100);
        return;
    }

    if (!isRobotAllowedToMove()) {
        stopMotors();
        return;
    }

    // --- Demo motor movement ---
    // Replace this part with your real line-following / robot behaviour code.
    if (millis() - lastDirectionChangeMs > 3000) {
        lastDirectionChangeMs = millis();
        movingForward = !movingForward;
    }

    if (movingForward) {
        mc.setSpeed(LEFT_MOTOR_CHANNEL, LEFT_DIR * MOTOR_SPEED);
        mc.setSpeed(RIGHT_MOTOR_CHANNEL, RIGHT_DIR * MOTOR_SPEED);
    } else {
        mc.setSpeed(LEFT_MOTOR_CHANNEL, -LEFT_DIR * MOTOR_SPEED);
        mc.setSpeed(RIGHT_MOTOR_CHANNEL, -RIGHT_DIR * MOTOR_SPEED);
    }
}