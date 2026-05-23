#include <Wire.h>
#include <Motoron.h>
#include <MiniMessenger.h>
#include "secrets.h"

MiniMessenger messenger;
MotoronI2C mc(16);

// Must match dashboard board name.
const char* BoardId = "Igor10";

// Motoron setup from robot_demo.ino
const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;

const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;

const int FORWARD_SPEED = -300;

// Mechanical switch + RGB LED from robot_demo.ino
const int switchPin = D8;

// Common cathode RGB LED.
// Only red is used. Green stays off.
const int r_led = D52;
const int g_led = D50;

// If the switch behaves backwards, change this to HIGH.
const int SWITCH_ENABLED_STATE = LOW;

// Safety state
bool wifiEnabled = false;
bool emergencyActive = false;
bool heartbeatTimeout = false;
bool localSwitchEnabled = false;

// Timing
unsigned long lastRegisterMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastBlinkMs = 0;
unsigned long lastStatusPrintMs = 0;

const unsigned long HEARTBEAT_TIMEOUT_MS = 1000;

// Switch debounce
bool lastSwitchReading = HIGH;
bool stableSwitchState = HIGH;
unsigned long lastDebounceMs = 0;
const unsigned long debounceDelay = 50;

bool wasConnected = false;


void driveMotors(int leftSpeed, int rightSpeed) {
  mc.setSpeed(LEFT_MOTOR_CHANNEL, LEFT_DIR * leftSpeed);
  mc.setSpeed(RIGHT_MOTOR_CHANNEL, RIGHT_DIR * rightSpeed);
}


void stopMotors() {
  mc.setSpeed(LEFT_MOTOR_CHANNEL, 0);
  mc.setSpeed(RIGHT_MOTOR_CHANNEL, 0);
}


void runRobot() {
  driveMotors(FORWARD_SPEED, FORWARD_SPEED);
}


void rgbOff() {
  digitalWrite(r_led, LOW);
  digitalWrite(g_led, LOW);
}


void setRed() {
  digitalWrite(r_led, HIGH);
  digitalWrite(g_led, LOW);
}


void setGreen() {
  rgbOff();
}


void blinkRed(unsigned long intervalMs) {
  if (millis() - lastBlinkMs >= intervalMs) {
    lastBlinkMs = millis();

    if (digitalRead(r_led) == HIGH) {
      rgbOff();
    } else {
      setRed();
    }
  }
}


void updateSwitch() {
  bool reading = digitalRead(switchPin);

  if (reading != lastSwitchReading) {
    lastDebounceMs = millis();
  }

  if (millis() - lastDebounceMs > debounceDelay) {
    if (reading != stableSwitchState) {
      stableSwitchState = reading;

      Serial.print("Mechanical switch: ");
      Serial.println(stableSwitchState == SWITCH_ENABLED_STATE ? "ENABLED" : "DISABLED");
    }
  }

  lastSwitchReading = reading;
  localSwitchEnabled = (stableSwitchState == SWITCH_ENABLED_STATE);
}


bool robotAllowedToMove() {
  return wifiEnabled && localSwitchEnabled && !emergencyActive && !heartbeatTimeout;
}


void updateLed() {
  if (robotAllowedToMove()) {
    rgbOff();
    return;
  }

  if (emergencyActive || heartbeatTimeout) {
    blinkRed(150);
    return;
  }

  blinkRed(500);
}


void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  char msg[128];

  size_t copyLen = (length < 127) ? length : 127;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';

  Serial.print("Message from ");
  Serial.print(metadata.fromBoardId);
  Serial.print(" to ");
  Serial.print(metadata.target);
  Serial.print(": ");
  Serial.println(msg);

  if (strstr(msg, "type=heartbeat")) {
    lastHeartbeatMs = millis();
    heartbeatTimeout = false;

    if (strstr(msg, "enable=1")) {
      wifiEnabled = true;
      emergencyActive = false;
      Serial.println("SAFETY: WiFi enabled by heartbeat");
    }
    else if (strstr(msg, "enable=0")) {
      wifiEnabled = false;
      emergencyActive = false;
      stopMotors();
      Serial.println("SAFETY: WiFi disabled by heartbeat");
    }
  }

  if (strstr(msg, "type=emergency") && strstr(msg, "enabled=true")) {
    wifiEnabled = false;
    emergencyActive = true;
    stopMotors();
    Serial.println("SAFETY: Emergency active");
  }

  if (strstr(msg, "type=disable") && strstr(msg, "enabled=false")) {
    wifiEnabled = false;
    emergencyActive = false;
    stopMotors();
    Serial.println("SAFETY: Individual disable active");
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(switchPin, INPUT_PULLUP);

  pinMode(r_led, OUTPUT);
  pinMode(g_led, OUTPUT);
  rgbOff();

  lastSwitchReading = digitalRead(switchPin);
  stableSwitchState = lastSwitchReading;
  localSwitchEnabled = (stableSwitchState == SWITCH_ENABLED_STATE);

  Serial.println("\n--- TEAM 10 WIFI + MECHANICAL KILL SWITCH STARTING ---");

  Wire1.begin();
  mc.setBus(&Wire1);

  mc.reinitialize();
  mc.clearResetFlag();
  mc.setCommandTimeoutMilliseconds(1000);

  mc.setMaxAcceleration(LEFT_MOTOR_CHANNEL, 200);
  mc.setMaxDeceleration(LEFT_MOTOR_CHANNEL, 300);

  mc.setMaxAcceleration(RIGHT_MOTOR_CHANNEL, 200);
  mc.setMaxDeceleration(RIGHT_MOTOR_CHANNEL, 300);

  stopMotors();

  messenger.onMessage(onMessage);
  messenger.begin(
    WIFI_SSID,
    WIFI_PASSWORD,
    BROKER_HOST,
    BROKER_PORT,
    GROUP_ID,
    BoardId
  );

  Serial.print("Initial mechanical switch: ");
  Serial.println(localSwitchEnabled ? "ENABLED" : "DISABLED");

  Serial.println("Network connecting...");
}


void loop() {
  messenger.loop();
  mc.resetCommandTimeout();

  updateSwitch();

  bool connected = messenger.isConnected();

  if (connected != wasConnected) {
    wasConnected = connected;
    Serial.print("Network/MQTT: ");
    Serial.println(connected ? "connected" : "disconnected");
  }

  if (!connected && millis() - lastStatusPrintMs >= 2000) {
    lastStatusPrintMs = millis();
    Serial.println("Waiting for WiFi/MQTT connection...");
  }

  if (connected && (millis() - lastRegisterMs >= 10000 || lastRegisterMs == 0)) {
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

  if (wifiEnabled && millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
    wifiEnabled = false;
    heartbeatTimeout = true;
    stopMotors();
    Serial.println("SAFETY: Heartbeat timeout");
  }

  updateLed();

  if (!robotAllowedToMove()) {
    stopMotors();
    return;
  }

  runRobot();
}