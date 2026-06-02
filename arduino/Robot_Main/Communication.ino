#include <MiniMessenger.h>
#include <string.h>
#include "secrets.h"

MiniMessenger messenger;

void stopMotors();
void abortPlanting();
bool robotAllowedToMove();

static bool wifiEnabled = false;
static bool emergencyActive = false;
static bool heartbeatTimeout = false;
static bool localSwitchEnabled = false;
static bool wasConnected = false;

static bool lastSwitchReading = HIGH;
static bool stableSwitchState = HIGH;

static unsigned long lastRegisterMs = 0;
static unsigned long lastHeartbeatMs = 0;
static unsigned long lastBlinkMs = 0;
static unsigned long lastStatusPrintMs = 0;
static unsigned long lastDebounceMs = 0;

static void setSafetyLedOff() {
  digitalWrite(RGB_RED_PIN, HIGH);
  digitalWrite(RGB_GREEN_PIN, HIGH);
}

static void blinkSafetyLedRed(unsigned long intervalMs) {
  if (millis() - lastBlinkMs < intervalMs) {
    return;
  }

  lastBlinkMs = millis();
  digitalWrite(RGB_GREEN_PIN, HIGH);
  digitalWrite(RGB_RED_PIN, digitalRead(RGB_RED_PIN) == LOW ? HIGH : LOW);
}

static void updateSafetySwitch() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastSwitchReading) {
    lastDebounceMs = millis();
  }

  if (millis() - lastDebounceMs > SWITCH_DEBOUNCE_MS && reading != stableSwitchState) {
    stableSwitchState = reading;

    Serial.print("Mechanical switch: ");
    Serial.println(stableSwitchState == SWITCH_ENABLED_STATE ? "ENABLED" : "DISABLED");
  }

  lastSwitchReading = reading;
  localSwitchEnabled = (stableSwitchState == SWITCH_ENABLED_STATE);
}

static void updateSafetyLed() {
  if (robotAllowedToMove()) {
    setSafetyLedOff();
    return;
  }

  if (emergencyActive || heartbeatTimeout) {
    blinkSafetyLedRed(150);
    return;
  }

  blinkSafetyLedRed(500);
}

static void registerWithServer() {
  char reg[80];
  snprintf(
    reg,
    sizeof(reg),
    "type=register team_id=%s board_id=%s",
    GROUP_ID,
    BOARD_ID
  );

  messenger.sendToBoard("server", reg);
  Serial.println("Registered with server.");
}

void onCommunicationMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  char msg[128];
  size_t copyLen = (length < sizeof(msg) - 1) ? length : sizeof(msg) - 1;

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
    } else if (strstr(msg, "enable=0")) {
      wifiEnabled = false;
      emergencyActive = false;
      stopMotors();
      abortPlanting();
      Serial.println("SAFETY: WiFi disabled by heartbeat");
    }
  }

  if (strstr(msg, "type=emergency") && strstr(msg, "enabled=true")) {
    wifiEnabled = false;
    emergencyActive = true;
    stopMotors();
    abortPlanting();
    Serial.println("SAFETY: Emergency active");
  }

  if (strstr(msg, "type=disable") && strstr(msg, "enabled=false")) {
    wifiEnabled = false;
    emergencyActive = false;
    stopMotors();
    abortPlanting();
    Serial.println("SAFETY: Individual disable active");
  }
}

void initRobotCommunication() {
  lastSwitchReading = digitalRead(BUTTON_PIN);
  stableSwitchState = lastSwitchReading;
  localSwitchEnabled = (stableSwitchState == SWITCH_ENABLED_STATE);

  messenger.onMessage(onCommunicationMessage);
  messenger.begin(
    WIFI_SSID,
    WIFI_PASSWORD,
    BROKER_HOST,
    BROKER_PORT,
    GROUP_ID,
    BOARD_ID
  );

  Serial.print("Initial mechanical switch: ");
  Serial.println(localSwitchEnabled ? "ENABLED" : "DISABLED");
  Serial.println("Network connecting...");
}

void updateRobotCommunication() {
  messenger.loop();
  updateSafetySwitch();

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

  if (connected && (lastRegisterMs == 0 || millis() - lastRegisterMs >= WIFI_REGISTER_INTERVAL_MS)) {
    lastRegisterMs = millis();
    registerWithServer();
  }

  if (wifiEnabled && millis() - lastHeartbeatMs > WIFI_HEARTBEAT_TIMEOUT_MS) {
    wifiEnabled = false;
    heartbeatTimeout = true;
    stopMotors();
    abortPlanting();
    Serial.println("SAFETY: Heartbeat timeout");
  }

  updateSafetyLed();
}

bool robotAllowedToMove() {
  return wifiEnabled && localSwitchEnabled && !emergencyActive && !heartbeatTimeout;
}

bool robotSafetyEmergencyActive() {
  return emergencyActive || heartbeatTimeout;
}
