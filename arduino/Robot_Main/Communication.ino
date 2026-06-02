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
static bool warnedLocalSafetyMode = false;

static bool lastSwitchReading = HIGH;
static bool stableSwitchState = HIGH;

static unsigned long lastRegisterMs = 0;
static unsigned long lastHeartbeatMs = 0;
static unsigned long lastBlinkMs = 0;
static unsigned long lastStatusPrintMs = 0;
static unsigned long lastDebounceMs = 0;

static bool airlockReplyReceived = false;
static bool airlockAccepted = false;
static char airlockReplyAirlock = '\0';

static bool fertilityReplyReceived = false;
static bool fertilityReplyFertile = false;
static bool fertilityReplyPlanted = true;
static int fertilityReplyX = -1;
static int fertilityReplyY = -1;

static bool reviveReplyReceived = false;
static bool reviveReplySuccess = false;

static bool teamStatusReceived = false;
static uint8_t teamStatus[6] = {0};

static bool occupancyMapReceived = false;
static uint8_t occupancyMapBytes[21] = {0};

static void formatTagId(uint32_t tagId, char *out, size_t outSize) {
  snprintf(out, outSize, "%08lX", (unsigned long)tagId);
}

static bool getValueForKey(const char *msg, const char *key, char *out, size_t outSize) {
  const char *start = strstr(msg, key);
  if (!start) return false;

  start += strlen(key);
  size_t i = 0;
  while (start[i] != '\0' && start[i] != ' ' && i < outSize - 1) {
    out[i] = start[i];
    i++;
  }

  out[i] = '\0';
  return i > 0;
}

static bool getBoolForKey(const char *msg, const char *key, bool defaultValue) {
  char value[12];
  if (!getValueForKey(msg, key, value, sizeof(value))) {
    return defaultValue;
  }

  return strcmp(value, "true") == 0 || strcmp(value, "1") == 0;
}

static int getIntForKey(const char *msg, const char *key, int defaultValue) {
  char value[12];
  if (!getValueForKey(msg, key, value, sizeof(value))) {
    return defaultValue;
  }

  return atoi(value);
}

static bool sendServerMessage(const char *msg) {
  if (!REQUIRE_SERVER_API) {
    Serial.print("LOCAL API MODE: would send ");
    Serial.println(msg);
    return true;
  }

  if (!messenger.isConnected()) {
    Serial.print("API send blocked, MQTT not connected: ");
    Serial.println(msg);
    return false;
  }

  bool sent = messenger.sendToBoard("server", msg);
  Serial.print(sent ? "API sent: " : "API send failed: ");
  Serial.println(msg);
  return sent;
}

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
  if (length == 6) {
    memcpy(teamStatus, payload, sizeof(teamStatus));
    teamStatusReceived = true;

    Serial.print("Team status: queueExit=");
    Serial.print(teamStatus[0]);
    Serial.print(" airlockBBusy=");
    Serial.print(teamStatus[1]);
    Serial.print(" queueEnter=");
    Serial.print(teamStatus[2]);
    Serial.print(" airlockABusy=");
    Serial.print(teamStatus[3]);
    Serial.print(" emergency=");
    Serial.print(teamStatus[4]);
    Serial.print(" reEntryRequested=");
    Serial.println(teamStatus[5]);
    return;
  }

  if (length == 21) {
    memcpy(occupancyMapBytes, payload, sizeof(occupancyMapBytes));
    occupancyMapReceived = true;
    Serial.println("Occupancy map update received.");
    return;
  }

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

  if (strstr(msg, "type=openAirlockReply")) {
    char airlockValue[4];
    airlockReplyReceived = true;
    airlockAccepted = getBoolForKey(msg, "accepted=", false);

    if (getValueForKey(msg, "airlock=", airlockValue, sizeof(airlockValue))) {
      airlockReplyAirlock = airlockValue[0];
    }

    Serial.print("AIRLOCK reply: airlock=");
    Serial.print(airlockReplyAirlock);
    Serial.print(" accepted=");
    Serial.println(airlockAccepted ? "true" : "false");
  }

  if (strstr(msg, "type=isFertileReply")) {
    fertilityReplyReceived = true;
    fertilityReplyFertile = getBoolForKey(msg, "fertile=", false);
    fertilityReplyPlanted = getBoolForKey(msg, "planted=", true);
    fertilityReplyX = getIntForKey(msg, "x=", -1);
    fertilityReplyY = getIntForKey(msg, "y=", -1);

    Serial.print("FERTILITY reply: fertile=");
    Serial.print(fertilityReplyFertile ? "true" : "false");
    Serial.print(" planted=");
    Serial.print(fertilityReplyPlanted ? "true" : "false");
    Serial.print(" x=");
    Serial.print(fertilityReplyX);
    Serial.print(" y=");
    Serial.println(fertilityReplyY);
  }

  if (strstr(msg, "type=reviveReply")) {
    char status[16];
    reviveReplyReceived = true;
    reviveReplySuccess = getValueForKey(msg, "status=", status, sizeof(status)) &&
                         strcmp(status, "success") == 0;

    Serial.print("REVIVE reply: success=");
    Serial.println(reviveReplySuccess ? "true" : "false");
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
  if (!REQUIRE_WIFI_SAFETY) {
    if (!warnedLocalSafetyMode) {
      Serial.println("LOCAL TEST MODE: WiFi heartbeat is not required for movement.");
      warnedLocalSafetyMode = true;
    }

    return localSwitchEnabled && !emergencyActive;
  }

  return wifiEnabled && localSwitchEnabled && !emergencyActive && !heartbeatTimeout;
}

bool robotSafetyEmergencyActive() {
  return emergencyActive || (REQUIRE_WIFI_SAFETY && heartbeatTimeout);
}

bool serverApiRequired() {
  return REQUIRE_SERVER_API;
}

bool requestOpenAirlock(char airlock, uint32_t tagId) {
  char tag[12];
  char msg[96];

  airlockReplyReceived = false;
  airlockAccepted = false;
  airlockReplyAirlock = airlock;

  if (!REQUIRE_SERVER_API) {
    airlockReplyReceived = true;
    airlockAccepted = true;
  }

  formatTagId(tagId, tag, sizeof(tag));
  snprintf(
    msg,
    sizeof(msg),
    "type=openAirlock airlock=%c tag_id=%s board_id=%s",
    airlock,
    tag,
    BOARD_ID
  );

  return sendServerMessage(msg);
}

bool airlockRequestAccepted() {
  return airlockReplyReceived && airlockAccepted;
}

bool airlockRequestRejected() {
  return airlockReplyReceived && !airlockAccepted;
}

bool requestFertilityCheck(uint32_t tagId) {
  char tag[12];
  char msg[88];

  fertilityReplyReceived = false;
  fertilityReplyFertile = false;
  fertilityReplyPlanted = true;
  fertilityReplyX = -1;
  fertilityReplyY = -1;

  if (!REQUIRE_SERVER_API) {
    fertilityReplyReceived = true;
    fertilityReplyFertile = true;
    fertilityReplyPlanted = false;
  }

  formatTagId(tagId, tag, sizeof(tag));
  snprintf(
    msg,
    sizeof(msg),
    "type=isFertile tag_id=%s board_id=%s",
    tag,
    BOARD_ID
  );

  return sendServerMessage(msg);
}

bool fertilityReplyReady() {
  return fertilityReplyReceived;
}

bool currentTagIsPlantable() {
  return fertilityReplyReceived && fertilityReplyFertile && !fertilityReplyPlanted;
}

int fertilityReplyCoordinateX() {
  return fertilityReplyX;
}

int fertilityReplyCoordinateY() {
  return fertilityReplyY;
}

bool reportSeedPlanted(uint32_t tagId) {
  char tag[12];
  char msg[88];

  formatTagId(tagId, tag, sizeof(tag));
  snprintf(
    msg,
    sizeof(msg),
    "type=seedPlanted tag_id=%s board_id=%s",
    tag,
    BOARD_ID
  );

  return sendServerMessage(msg);
}

bool requestRevive(int targetTeam, const char *targetBoard) {
  char msg[96];

  reviveReplyReceived = false;
  reviveReplySuccess = false;

  if (!REQUIRE_SERVER_API) {
    reviveReplyReceived = true;
    reviveReplySuccess = true;
  }

  snprintf(
    msg,
    sizeof(msg),
    "type=reviveRequest target_team=%d target_board=%s",
    targetTeam,
    targetBoard
  );

  return sendServerMessage(msg);
}

bool reviveReplyReady() {
  return reviveReplyReceived;
}

bool reviveSucceeded() {
  return reviveReplyReceived && reviveReplySuccess;
}

bool requestOccupancyMap() {
  char msg[48];
  snprintf(msg, sizeof(msg), "type=getMap board_id=%s", BOARD_ID);
  return sendServerMessage(msg);
}

bool occupancyMapIsReady() {
  return occupancyMapReceived;
}

uint8_t getOccupancyMapCell(int x, int y) {
  if (!occupancyMapReceived || x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT) {
    return 3;
  }

  int cellIndex = y * GRID_WIDTH + x;
  int bitIndex = cellIndex * 2;
  int byteIndex = bitIndex / 8;
  int bitOffset = bitIndex % 8;

  return (occupancyMapBytes[byteIndex] >> bitOffset) & 0x03;
}
