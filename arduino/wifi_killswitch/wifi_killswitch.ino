#include <Wire.h>
#include <Motoron.h>
#include <MiniMessenger.h>
#include "secrets.h"

/*
 * GIGA Motoron WiFi Kill Switch Test
 *
 * Hardware:
 * - Arduino GIGA
 * - 1 Motoron motor driver on Wire1
 * - 2 DC motors
 * - 1 red LED
 *
 * Behaviour:
 * - Upload / waiting / disabled  -> motors stop, red LED slow blink
 * - Dashboard enable             -> motors run, red LED ON
 * - Dashboard disable            -> motors stop, red LED slow blink
 * - Emergency command            -> motors stop, red LED fast blink
 * - Heartbeat timeout            -> motors stop, red LED fast blink
 */

MiniMessenger messenger;

// Motoron address confirmed by I2C scanner:
// 0x10 hex = 16 decimal
MotoronI2C mc(16);

// --- Board configuration ---
const char* BoardId = "Igor10";

// --- Motor configuration ---
const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;

const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;

const int MOTOR_SPEED = 300;

// --- LED configuration ---
const int RED_LED_PIN = 9;

// --- Safety state ---
bool safetyEnabled = false;
bool emergencyActive = false;
bool heartbeatTimeout = false;

// --- Timing variables ---
unsigned long lastRegisterMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastBlinkMs = 0;

const unsigned long HEARTBEAT_TIMEOUT_MS = 1000;


// Stop both motors immediately
void stopMotors() {
    mc.setSpeed(LEFT_MOTOR_CHANNEL, 0);
    mc.setSpeed(RIGHT_MOTOR_CHANNEL, 0);
}


// Simple motor movement for testing
void runMotors() {
    mc.setSpeed(LEFT_MOTOR_CHANNEL, LEFT_DIR * MOTOR_SPEED);
    mc.setSpeed(RIGHT_MOTOR_CHANNEL, RIGHT_DIR * MOTOR_SPEED);
}


// LED status indicator
void updateLed() {
    // Emergency or heartbeat timeout: fast blink
    if (emergencyActive || heartbeatTimeout) {
        if (millis() - lastBlinkMs > 150) {
            lastBlinkMs = millis();
            digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
        }
        return;
    }

    // Enabled and safe: LED ON
    if (safetyEnabled) {
        digitalWrite(RED_LED_PIN, HIGH);
        return;
    }

    // Waiting / disabled / motor not allowed: slow blink
    if (millis() - lastBlinkMs > 500) {
        lastBlinkMs = millis();
        digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
    }
}


// Runs every time an MQTT message arrives
void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
    char msg[128];

    size_t copyLen = (length < 127) ? length : 127;
    memcpy(msg, payload, copyLen);
    msg[copyLen] = '\0';

    Serial.print("Message received: ");
    Serial.println(msg);

    // Heartbeat message from server
    if (strstr(msg, "type=heartbeat")) {
        lastHeartbeatMs = millis();
        heartbeatTimeout = false;

        if (strstr(msg, "enable=1")) {
            safetyEnabled = true;
            emergencyActive = false;
            Serial.println("SAFETY: Enabled by heartbeat");
        }
        else if (strstr(msg, "enable=0")) {
            safetyEnabled = false;
            emergencyActive = false;
            stopMotors();
            Serial.println("SAFETY: Disabled by heartbeat");
        }
    }

    // Emergency stop
    if (strstr(msg, "type=emergency enabled=true")) {
        safetyEnabled = false;
        emergencyActive = true;
        stopMotors();
        Serial.println("SAFETY: Emergency active");
    }

    // Individual disable
    if (strstr(msg, "type=disable enabled=false")) {
        safetyEnabled = false;
        emergencyActive = false;
        stopMotors();
        Serial.println("SAFETY: Individual disable active");
    }
}


void setup() {
    Serial.begin(115200);

    pinMode(RED_LED_PIN, OUTPUT);
    digitalWrite(RED_LED_PIN, LOW);

    Serial.println("\n--- GIGA MOTORON WIFI KILL SWITCH TEST STARTING ---");

    // --- Motoron setup on Wire1 ---
    Wire1.begin();
    mc.setBus(&Wire1);

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

    messenger.begin(
        WIFI_SSID,
        WIFI_PASSWORD,
        BROKER_HOST,
        BROKER_PORT,
        GROUP_ID,
        BoardId
    );

    Serial.println("Network connecting...");
}


void loop() {
    // Required for receiving MQTT messages
    messenger.loop();

    // Register with server every 10 seconds
    if (millis() - lastRegisterMs > 10000 || lastRegisterMs == 0) {
        lastRegisterMs = millis();

        char reg[64];

        snprintf(
            reg,
            sizeof(reg),
            "type=register team_id=%s board_id=%s",
            GROUP_ID,
            BoardId
        );

        messenger.sendToBoard("server", reg);
        Serial.println("Registered with server.");
    }

    // Heartbeat timeout check
    if (safetyEnabled && millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
        safetyEnabled = false;
        heartbeatTimeout = true;
        stopMotors();
        Serial.println("SAFETY: Heartbeat timeout");
    }

    // LED must update continuously
    updateLed();

    // Safety gate
    if (!safetyEnabled) {
        stopMotors();
        return;
    }

    // Motors only run when WiFi safety is enabled
    runMotors();
}