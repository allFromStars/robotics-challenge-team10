#include <Wire.h>
#include "ICM_20948.h"
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <MiniMessenger.h>
#include <string.h>
#include "../../Robot_Main/secrets.h"

// ============================================================
// Task 2: Base Exit Route + IMU Turns + RFID-Gated Navigation
// ============================================================
//
// Route:
// deployment area
// -> follow straight line
// -> detect base junction
// -> IMU right 90
// -> reacquire lower exit route line
// -> follow straight
// -> IMU left 90
// -> reacquire B tag line
// -> follow slowly
// -> detect RFID tag
// -> request Airlock B from server
// -> wait for Airlock B accepted
// -> keep following to next junction
// -> IMU left 90
// -> follow to next junction
// -> IMU right 90
// -> follow for 3 seconds as a temporary stop case
//
// This version uses the ICM-20948 gyroscope yaw integration for turning.
// It keeps the early line-stop logic during turns.

// --------------------
// Motoron setup
// --------------------

MotoronI2C mc(16);
MiniMessenger messenger;

const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;

const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;
const int FORWARD_SIGN = 1;

// Mechanical button wiring:
// one side of the button -> GND
// other side -> D8
// INPUT_PULLUP means released = HIGH, pressed = LOW.
const int CONTROL_BUTTON_PIN = D8;
const int CONTROL_BUTTON_PRESSED = LOW;
const unsigned long DEBOUNCE_DELAY_MS = 50;

// --------------------
// MiniMessenger setup
// --------------------

const char TASK2_BOARD_ID[] = "treadzeppelin";

const unsigned long WIFI_REGISTER_INTERVAL_MS = 10000;
const unsigned long NETWORK_STATUS_PRINT_MS = 2000;
const unsigned long SERVER_REPLY_TIMEOUT_MS = 5000;
const unsigned long SERVER_REQUEST_RETRY_MS = 1500;

bool wasNetworkConnected = false;
unsigned long lastRegisterMs = 0;
unsigned long lastNetworkStatusPrintMs = 0;
unsigned long serverRequestStartedMs = 0;
unsigned long lastServerRequestMs = 0;

bool airlockReplyReceived = false;
bool airlockAccepted = false;
char requestedAirlock = '\0';
char airlockReplyAirlock = '\0';
char activeAirlockTagId[16] = "";

// --------------------
// IMU setup
// --------------------

ICM_20948_I2C myICM;

float yaw = 0.0;
float gyroBiasZ = 0.0;
float lastImuDt = 0.01;
unsigned long lastImuUpdateMicros = 0;

bool imuTurnStarted = false;
float imuTurnStartYaw = 0.0;
float imuTurnTargetYaw = 0.0;
float lastTurnError = 0.0;
float turnIntegral = 0.0;
unsigned long turnSettleStartMs = 0;

// --------------------
// Speed tuning
// --------------------

const int SPEED_CAREFUL = 140;
const int SPEED_LINE = 220;
const int SPEED_FAST = 280;
const int MAX_SPEED = 360;

float Kp = 0.060;
float Kd = 0.12;
// --------------------
// IMU turn tuning
// --------------------
//
// Positive angle should match the right turn on this robot.
// If the robot turns the wrong way, swap RIGHT_TURN_ANGLE_DEG and
// LEFT_TURN_ANGLE_DEG signs.

const float RIGHT_TURN_ANGLE_DEG = -90.0;
const float LEFT_TURN_ANGLE_DEG = 90.0;

float turnKp = 19.5;
float turnKd = 6.5;
float turnKi = 1.8;

const int TURN_MOTOR_MAX_LIMIT = 280;
const int TURN_MOTOR_MIN_LIMIT = 180;
const float TURN_INTEGRAL_LIMIT = 1000.0;

const float GYRO_DEADBAND_DPS = 0.1;
const float TURN_ANGLE_TOLERANCE_DEG = 1.0;
const unsigned long TURN_SETTLE_MS = 80;

// --------------------
// IR reflectance array
// --------------------

const uint8_t SensorCount = 9;
const uint8_t sensorPins[SensorCount] = {
  22, 23, 24, 25, 26, 27, 28, 29, 30
};

uint16_t sensorValues[SensorCount];
uint16_t calibratedValues[SensorCount];
uint16_t sensorMin[SensorCount];
uint16_t sensorMax[SensorCount];

const uint16_t timeout = 2500;
const unsigned long CALIBRATION_TIME_MS = 12000;

const int LINE_CENTER = 4000;
const uint16_t LINE_THRESHOLD = 400;
const uint16_t LINE_TOTAL_THRESHOLD = 50;

const int BASE_JUNCTION_ACTIVE_SENSOR_COUNT = 6;
const unsigned long BASE_JUNCTION_STABLE_MS = 100;

uint16_t lastLinePosition = LINE_CENTER;
int lastError = 0;
int activeSensorCount = 0;
bool lineDetected = false;
bool baseJunctionDetected = false;

unsigned long baseJunctionFirstSeenMs = 0;

// --------------------
// RFID setup
// --------------------

#define RFID_ADDR 0x28
MFRC522_I2C rfid(RFID_ADDR, -1, &Wire1);

// Only needed when ACCEPT_ANY_RFID_AS_B_TAG is false.
const String TARGET_B_UID = "REPLACE_WITH_B_UID";

const bool ACCEPT_ANY_RFID_AS_B_TAG = true;

String latestUid = "";
bool anyRfidDetected = false;
bool bTagDetected = false;

// --------------------
// Communication helpers
// --------------------

bool getValueForKey(const char *msg, const char *key, char *out, size_t outSize) {
  const char *start = strstr(msg, key);
  if (!start) {
    return false;
  }

  start += strlen(key);
  size_t i = 0;

  while (start[i] != '\0' && start[i] != ' ' && i < outSize - 1) {
    out[i] = start[i];
    i++;
  }

  out[i] = '\0';
  return i > 0;
}

bool getBoolForKey(const char *msg, const char *key, bool defaultValue) {
  char value[12];

  if (!getValueForKey(msg, key, value, sizeof(value))) {
    return defaultValue;
  }

  return strcmp(value, "true") == 0 || strcmp(value, "1") == 0;
}

void onCommunicationMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  if (length == 6 || length == 21) {
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
}

void registerWithServer() {
  char reg[80];
  snprintf(
    reg,
    sizeof(reg),
    "type=register team_id=%s board_id=%s",
    GROUP_ID,
    TASK2_BOARD_ID
  );

  bool sent = messenger.sendToBoard("server", reg);
  Serial.println(sent ? "Register sent to server." : "Register send failed.");
}

void initTask2Communication() {
  messenger.onMessage(onCommunicationMessage);
  messenger.begin(
    WIFI_SSID,
    WIFI_PASSWORD,
    BROKER_HOST,
    BROKER_PORT,
    GROUP_ID,
    TASK2_BOARD_ID
  );

  Serial.println("Network connecting...");
}

void updateTask2Communication() {
  messenger.loop();

  bool connected = messenger.isConnected();

  if (connected != wasNetworkConnected) {
    wasNetworkConnected = connected;
    Serial.print("Network/MQTT: ");
    Serial.println(connected ? "connected" : "disconnected");
  }

  if (!connected && millis() - lastNetworkStatusPrintMs >= NETWORK_STATUS_PRINT_MS) {
    lastNetworkStatusPrintMs = millis();
    Serial.println("Waiting for WiFi/MQTT connection...");
  }

  if (connected && (lastRegisterMs == 0 || millis() - lastRegisterMs >= WIFI_REGISTER_INTERVAL_MS)) {
    lastRegisterMs = millis();
    registerWithServer();
  }
}

bool requestOpenAirlock(char airlock, const char *tagId) {
  char msg[96];

  airlockReplyReceived = false;
  airlockAccepted = false;
  requestedAirlock = airlock;
  airlockReplyAirlock = airlock;

  if (!messenger.isConnected()) {
    Serial.println("Airlock request blocked: MQTT not connected.");
    return false;
  }

  snprintf(
    msg,
    sizeof(msg),
    "type=openAirlock airlock=%c tag_id=%s board_id=%s",
    airlock,
    tagId,
    TASK2_BOARD_ID
  );

  bool sent = messenger.sendToBoard("server", msg);
  Serial.print(sent ? "Airlock request sent: " : "Airlock request send failed: ");
  Serial.println(msg);
  return sent;
}

bool airlockRequestAccepted() {
  return airlockReplyReceived && airlockAccepted && airlockReplyAirlock == requestedAirlock;
}

bool airlockRequestRejected() {
  return airlockReplyReceived && !airlockAccepted;
}

// --------------------
// Task 2 state machine
// --------------------

enum Task2State {
  T2_IDLE,
  T2_LEAVE_DEPLOYMENT_AREA,
  T2_FOLLOW_DEPLOYMENT_LINE,
  T2_TURN_RIGHT_TO_EXIT_ROUTE,
  T2_REACQUIRE_EXIT_ROUTE,
  T2_FOLLOW_EXIT_ROUTE_STRAIGHT,
  T2_TURN_LEFT_TO_B_LINE,
  T2_REACQUIRE_B_LINE,
  T2_APPROACH_B_TAG,
  T2_REQUEST_AIRLOCK_B,
  T2_WAIT_AIRLOCK_B,
  T2_FOLLOW_AFTER_B_TAG_TO_JUNCTION,
  T2_TURN_LEFT_AFTER_B_TAG,
  T2_REACQUIRE_AFTER_B_TAG_LEFT,
  T2_FOLLOW_TO_SECOND_JUNCTION,
  T2_TURN_RIGHT_AFTER_SECOND_JUNCTION,
  T2_REACQUIRE_AFTER_SECOND_RIGHT,
  T2_FINAL_FOLLOW_TEMP_STOP,
  T2_DONE,
  T2_FAILSAFE_STOP
};

Task2State task2State = T2_IDLE;

unsigned long stateStartMs = 0;
unsigned long taskStartMs = 0;
unsigned long lineLostStartMs = 0;
unsigned long lastDebugPrintMs = 0;
unsigned long reacquireCenteredStartMs = 0;
bool task2Running = false;
bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;

// --------------------
// Route timing tuning
// --------------------

const unsigned long EXIT_ROUTE_JUNCTION_IGNORE_MS = 800;
const unsigned long EXIT_ROUTE_MAX_STRAIGHT_MS = 15000;

const unsigned long FINAL_FOLLOW_TEMP_STOP_MS = 3000;
const unsigned long TASK2_TIMEOUT_MS = 70000;
const unsigned long LOST_LINE_TIMEOUT_MS = 1200;

const unsigned long LEAVE_DEPLOYMENT_MIN_MS = 200;
const unsigned long REACQUIRE_TIMEOUT_MS = 2500;

// A line is considered safely reacquired only when it is visible,
// not a wide junction, and not at the extreme edge of the sensor array.
const int REACQUIRE_MAX_ABS_ERROR = 1800;
const int REACQUIRE_CENTER_MAX_ABS_ERROR = 1200;
const unsigned long REACQUIRE_CENTER_STABLE_MS = 250;

// Ignore line detections briefly at the start of each turn, so the robot
// does not immediately stop on the line or junction it is leaving.
const unsigned long RIGHT_TURN_IGNORE_LINE_MS = 600;
const unsigned long TURN_IGNORE_LINE_MS = 250;

// Early line-stop is only allowed after the robot has already turned a
// reasonable amount. Right turn starts from a junction, so it uses IMU only.
const float TURN_MIN_EARLY_STOP_DEG = 45.0;

// --------------------
// Motor helpers
// --------------------

int clampMagnitude(int speed) {
  if (speed < 0) {
    return 0;
  }

  if (speed > MAX_SPEED) {
    return MAX_SPEED;
  }

  return speed;
}

void driveMotors(int leftLogicalSpeed, int rightLogicalSpeed) {
  mc.setSpeed(LEFT_MOTOR_CHANNEL, LEFT_DIR * leftLogicalSpeed);
  mc.setSpeed(RIGHT_MOTOR_CHANNEL, RIGHT_DIR * rightLogicalSpeed);
}

void stopMotors() {
  mc.setSpeed(LEFT_MOTOR_CHANNEL, 0);
  mc.setSpeed(RIGHT_MOTOR_CHANNEL, 0);
}

void driveForwardMagnitudes(int leftMagnitude, int rightMagnitude) {
  leftMagnitude = clampMagnitude(leftMagnitude);
  rightMagnitude = clampMagnitude(rightMagnitude);

  driveMotors(FORWARD_SIGN * leftMagnitude, FORWARD_SIGN * rightMagnitude);
}

void turnInPlace(int direction, int speed) {
  if (direction < 0) {
    driveMotors(speed, -speed);
  } else {
    driveMotors(-speed, speed);
  }
}

// --------------------
// IMU helpers
// --------------------

void updateImuYaw() {
  if (!myICM.dataReady()) {
    return;
  }

  myICM.getAGMT();

  unsigned long currentMicros = micros();

  if (lastImuUpdateMicros == 0) {
    lastImuUpdateMicros = currentMicros;
    return;
  }

  lastImuDt = (currentMicros - lastImuUpdateMicros) / 1000000.0;
  lastImuUpdateMicros = currentMicros;

  if (lastImuDt <= 0.0 || lastImuDt > 0.2) {
    return;
  }

  float gyroZ = myICM.gyrZ() - gyroBiasZ;

  if (abs(gyroZ) > GYRO_DEADBAND_DPS) {
    yaw += gyroZ * lastImuDt;
  }
}

void printCountdown(const char* message, int seconds) {
  Serial.println(message);

  for (int remaining = seconds; remaining > 0; remaining--) {
    Serial.print("Starting in ");
    Serial.print(remaining);
    Serial.println("...");
    delay(1000);
  }
}

void calibrateGyroZ() {
  Serial.println();
  Serial.println("=== IMU GYRO CALIBRATION ===");
  printCountdown("Keep the robot completely still.", 3);
  Serial.println("Reading gyro Z bias: 1000 samples");

  gyroBiasZ = 0.0;
  int samples = 0;

  while (samples < 1000) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      gyroBiasZ += myICM.gyrZ();
      samples++;

      if (samples % 50 == 0) {
        Serial.print("IMU samples: ");
        Serial.print(samples);
        Serial.println(" / 1000");
      }
    }

    delay(5);
  }

  gyroBiasZ /= 1000.0;

  Serial.println();
  Serial.print("Gyro Z bias = ");
  Serial.println(gyroBiasZ, 4);
  Serial.println("IMU calibration complete.");

  yaw = 0.0;
  lastImuUpdateMicros = micros();
}

void initImu() {
  bool initialized = false;

  while (!initialized) {
    myICM.begin(Wire, 0);

    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.print("ICM20948 connection failed: ");
      Serial.println(myICM.statusString());
      delay(500);
    } else {
      initialized = true;
    }
  }

  calibrateGyroZ();
}

void resetImuTurn() {
  imuTurnStarted = false;
  imuTurnStartYaw = yaw;
  turnSettleStartMs = 0;
  lastTurnError = 0.0;
  turnIntegral = 0.0;
}

bool runImuTurn(float targetRelativeAngleDeg) {
  if (!imuTurnStarted) {
    imuTurnStartYaw = yaw;
    imuTurnTargetYaw = yaw + targetRelativeAngleDeg;
    lastTurnError = targetRelativeAngleDeg;
    turnIntegral = 0.0;
    turnSettleStartMs = 0;
    imuTurnStarted = true;

    Serial.print("IMU turn start. targetRelative=");
    Serial.print(targetRelativeAngleDeg, 1);
    Serial.print(" targetYaw=");
    Serial.println(imuTurnTargetYaw, 1);
  }

  float error = imuTurnTargetYaw - yaw;

  if (abs(error) <= TURN_ANGLE_TOLERANCE_DEG) {
    if (turnSettleStartMs == 0) {
      turnSettleStartMs = millis();
    }

    if (millis() - turnSettleStartMs >= TURN_SETTLE_MS) {
      stopMotors();
      resetImuTurn();
      return true;
    }
  } else {
    turnSettleStartMs = 0;
  }

  float derivative = 0.0;

  if (lastImuDt > 0.0) {
    turnIntegral += error * lastImuDt;
    turnIntegral = constrain(turnIntegral, -TURN_INTEGRAL_LIMIT, TURN_INTEGRAL_LIMIT);
    derivative = (error - lastTurnError) / lastImuDt;
  }

  lastTurnError = error;

  float turnOutput = turnKp * error + turnKi * turnIntegral + turnKd * derivative;

  if (abs(turnOutput) < TURN_MOTOR_MIN_LIMIT && abs(error) > TURN_ANGLE_TOLERANCE_DEG) {
    turnOutput = (turnOutput > 0.0) ? TURN_MOTOR_MIN_LIMIT : -TURN_MOTOR_MIN_LIMIT;
  }

  turnOutput = constrain(turnOutput, -TURN_MOTOR_MAX_LIMIT, TURN_MOTOR_MAX_LIMIT);

  int leftSpeed = (int)(FORWARD_SIGN * turnOutput);
  int rightSpeed = (int)(-FORWARD_SIGN * turnOutput);

  driveMotors(leftSpeed, rightSpeed);

  return false;
}

bool imuTurnPassedAngle(float minAngleDeg) {
  return abs(yaw - imuTurnStartYaw) >= minAngleDeg;
}

// --------------------
// IR reading
// --------------------

void readQTR_RC() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }

  delayMicroseconds(10);

  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorValues[i] = timeout;
  }

  unsigned long startTime = micros();
  bool allDone = false;

  while (!allDone && (micros() - startTime < timeout)) {
    allDone = true;

    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] == timeout) {
        if (digitalRead(sensorPins[i]) == LOW) {
          sensorValues[i] = micros() - startTime;
        } else {
          allDone = false;
        }
      }
    }
  }
}

void calibrateSensors() {
  Serial.println();
  Serial.println("=== IR LINE SENSOR CALIBRATION ===");
  printCountdown("Move the sensor array over white floor and black line when the countdown ends.", 3);
  Serial.println("IR calibration running for 12 seconds...");

  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorMin[i] = timeout;
    sensorMax[i] = 0;
  }

  unsigned long startTime = millis();
  unsigned long lastSecondPrintMs = 0;

  while (millis() - startTime < CALIBRATION_TIME_MS) {
    readQTR_RC();

    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] < sensorMin[i]) {
        sensorMin[i] = sensorValues[i];
      }

      if (sensorValues[i] > sensorMax[i]) {
        sensorMax[i] = sensorValues[i];
      }
    }

    unsigned long elapsedMs = millis() - startTime;

    if (elapsedMs - lastSecondPrintMs >= 1000) {
      lastSecondPrintMs = elapsedMs;
      int remainingSeconds = (CALIBRATION_TIME_MS - elapsedMs + 999) / 1000;
      Serial.print("IR calibration remaining: ");
      Serial.print(remainingSeconds);
      Serial.println("s");
    }

    delay(10);
  }

  Serial.println("IR calibration complete.");
}

void calculateCalibratedValues() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorMax[i] <= sensorMin[i]) {
      calibratedValues[i] = 0;
      continue;
    }

    long value = sensorValues[i];
    value = ((value - sensorMin[i]) * 1000L) / (sensorMax[i] - sensorMin[i]);
    value = constrain(value, 0, 1000);
    calibratedValues[i] = value;
  }
}

uint16_t readLinePosition() {
  uint32_t weightedSum = 0;
  uint32_t total = 0;
  activeSensorCount = 0;

  for (uint8_t i = 0; i < SensorCount; i++) {
    uint16_t value = calibratedValues[i];

    if (value < LINE_THRESHOLD) {
      value = 0;
    } else {
      activeSensorCount++;
    }

    weightedSum += (uint32_t)value * (i * 1000);
    total += value;
  }

  lineDetected = total > LINE_TOTAL_THRESHOLD;
  baseJunctionDetected = activeSensorCount >= BASE_JUNCTION_ACTIVE_SENSOR_COUNT;

  if (!lineDetected) {
    return lastLinePosition;
  }

  lastLinePosition = weightedSum / total;
  return lastLinePosition;
}

void updateLineState() {
  readQTR_RC();
  calculateCalibratedValues();
  readLinePosition();
}

bool baseJunctionStable() {
  if (!baseJunctionDetected) {
    baseJunctionFirstSeenMs = 0;
    return false;
  }

  if (baseJunctionFirstSeenMs == 0) {
    baseJunctionFirstSeenMs = millis();
    return false;
  }

  return millis() - baseJunctionFirstSeenMs >= BASE_JUNCTION_STABLE_MS;
}

bool lineReacquired() {
  int currentError = (int)lastLinePosition - LINE_CENTER;

  return lineDetected &&
         !baseJunctionDetected &&
         abs(currentError) <= REACQUIRE_MAX_ABS_ERROR &&
         activeSensorCount >= 1 &&
         activeSensorCount <= 5;
}

bool lineCenteredAfterReacquire() {
  bool centered = lineDetected &&
                  !baseJunctionDetected &&
                  abs(lastError) <= REACQUIRE_CENTER_MAX_ABS_ERROR &&
                  activeSensorCount >= 1 &&
                  activeSensorCount <= 5;

  if (!centered) {
    reacquireCenteredStartMs = 0;
    return false;
  }

  if (reacquireCenteredStartMs == 0) {
    reacquireCenteredStartMs = millis();
    return false;
  }

  return millis() - reacquireCenteredStartMs >= REACQUIRE_CENTER_STABLE_MS;
}

bool validLineForEarlyTurnStop(unsigned long ignoreLineMs) {
  return millis() - stateStartMs > ignoreLineMs &&
         lineDetected &&
         !baseJunctionDetected &&
         activeSensorCount >= 1 &&
         activeSensorCount <= 5;
}

// --------------------
// RFID reading
// --------------------

void updateRfid() {
  anyRfidDetected = false;
  bTagDetected = false;

  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return;
  }

  latestUid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) {
      latestUid += "0";
    }
    latestUid += String(rfid.uid.uidByte[i], HEX);
  }

  latestUid.toUpperCase();
  Serial.print("RFID UID: ");
  Serial.println(latestUid);

  anyRfidDetected = true;
  bTagDetected = ACCEPT_ANY_RFID_AS_B_TAG || latestUid == TARGET_B_UID;

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// --------------------
// Movement behaviours
// --------------------

void followLinePD(int baseSpeed) {
  if (!lineDetected) {
    if (lastError < 0) {
      turnInPlace(-1, SPEED_LINE);
    } else {
      turnInPlace(1, SPEED_LINE);
    }
    return;
  }

  int error = (int)lastLinePosition - LINE_CENTER;
  int derivative = error - lastError;
  int correction = (int)(Kp * error + Kd * derivative);

  int leftMagnitude = baseSpeed + correction;
  int rightMagnitude = baseSpeed - correction;

  driveForwardMagnitudes(leftMagnitude, rightMagnitude);
  lastError = error;
}

void enterState(Task2State nextState) {
  task2State = nextState;
  stateStartMs = millis();
  baseJunctionFirstSeenMs = 0;
  reacquireCenteredStartMs = 0;
  lineLostStartMs = 0;

  Serial.print("Task 2 state -> ");
  Serial.println((int)task2State);
}

void enterFailsafe(const char* reason) {
  stopMotors();
  Serial.print("FAILSAFE: ");
  Serial.println(reason);
  enterState(T2_FAILSAFE_STOP);
}

void resetTask2Run() {
  lastError = 0;
  lastLinePosition = LINE_CENTER;
  baseJunctionFirstSeenMs = 0;
  reacquireCenteredStartMs = 0;
  lineLostStartMs = 0;
  latestUid = "";
  anyRfidDetected = false;
  bTagDetected = false;
  activeAirlockTagId[0] = '\0';
  airlockReplyReceived = false;
  airlockAccepted = false;
  requestedAirlock = '\0';
  airlockReplyAirlock = '\0';
  serverRequestStartedMs = 0;
  lastServerRequestMs = 0;
  resetImuTurn();
  taskStartMs = millis();
}

void updateLineLostTimer() {
  if (lineDetected) {
    lineLostStartMs = 0;
    return;
  }

  if (lineLostStartMs == 0) {
    lineLostStartMs = millis();
  }
}

bool task2TimedOut() {
  return taskStartMs > 0 && millis() - taskStartMs > TASK2_TIMEOUT_MS;
}

bool lineLostTooLong() {
  return lineLostStartMs > 0 && millis() - lineLostStartMs > LOST_LINE_TIMEOUT_MS;
}

void printDebugStatus() {
  if (millis() - lastDebugPrintMs < 200) {
    return;
  }

  lastDebugPrintMs = millis();

  Serial.print("state=");
  Serial.print((int)task2State);
  Serial.print(" line=");
  Serial.print(lineDetected ? 1 : 0);
  Serial.print(" active=");
  Serial.print(activeSensorCount);
  Serial.print(" error=");
  Serial.print(lastError);
  Serial.print(" junction=");
  Serial.print(baseJunctionDetected ? 1 : 0);
  Serial.print(" yaw=");
  Serial.print(yaw, 1);
  Serial.print(" targetYaw=");
  Serial.print(imuTurnTargetYaw, 1);
  Serial.print(" uid=");
  Serial.print(latestUid);
  Serial.print(" anyRfid=");
  Serial.print(anyRfidDetected ? 1 : 0);
  Serial.print(" b=");
  Serial.println(bTagDetected ? 1 : 0);
}

// --------------------
// Task 2 state machine
// --------------------

void runTask2() {
  updateImuYaw();
  updateLineState();
  updateRfid();
  updateLineLostTimer();
  printDebugStatus();

  if (task2TimedOut() &&
      task2State != T2_IDLE &&
      task2State != T2_DONE &&
      task2State != T2_FAILSAFE_STOP) {
    enterFailsafe("Task 2 exceeded timeout");
    return;
  }

  switch (task2State) {
    case T2_IDLE:
      stopMotors();
      break;

    case T2_LEAVE_DEPLOYMENT_AREA:
      driveForwardMagnitudes(SPEED_CAREFUL, SPEED_CAREFUL);

      if (lineDetected && millis() - stateStartMs > LEAVE_DEPLOYMENT_MIN_MS) {
        enterState(T2_FOLLOW_DEPLOYMENT_LINE);
      }
      break;

    case T2_FOLLOW_DEPLOYMENT_LINE:
      followLinePD(SPEED_FAST);

      if (baseJunctionStable()) {
        enterState(T2_TURN_RIGHT_TO_EXIT_ROUTE);
      }

      if (lineLostTooLong()) {
        enterFailsafe("Line lost on deployment line");
      }
      break;

    case T2_TURN_RIGHT_TO_EXIT_ROUTE:
      if (runImuTurn(RIGHT_TURN_ANGLE_DEG)) {
        lastError = 0;
        lastLinePosition = LINE_CENTER;
        enterState(T2_REACQUIRE_EXIT_ROUTE);
      }
      break;

    case T2_REACQUIRE_EXIT_ROUTE:
      followLinePD(SPEED_LINE);

      if (lineCenteredAfterReacquire()) {
        enterState(T2_FOLLOW_EXIT_ROUTE_STRAIGHT);
      }

      if (millis() - stateStartMs > REACQUIRE_TIMEOUT_MS) {
        enterFailsafe("Could not reacquire exit route after right turn");
      }
      break;

    case T2_FOLLOW_EXIT_ROUTE_STRAIGHT:
      followLinePD(SPEED_LINE);

      if (millis() - stateStartMs > EXIT_ROUTE_JUNCTION_IGNORE_MS &&
          baseJunctionStable()) {
        enterState(T2_TURN_LEFT_TO_B_LINE);
      }

      if (millis() - stateStartMs > EXIT_ROUTE_MAX_STRAIGHT_MS) {
        enterFailsafe("Could not find left-turn junction on exit route");
      }

      if (lineLostTooLong()) {
        enterFailsafe("Line lost on exit route straight");
      }
      break;

    case T2_TURN_LEFT_TO_B_LINE:
      if (imuTurnPassedAngle(TURN_MIN_EARLY_STOP_DEG) &&
          validLineForEarlyTurnStop(TURN_IGNORE_LINE_MS)) {
        stopMotors();
        resetImuTurn();
        lastError = 0;
        baseJunctionFirstSeenMs = 0;
        enterState(T2_APPROACH_B_TAG);
        break;
      }

      if (runImuTurn(LEFT_TURN_ANGLE_DEG)) {
        lastError = 0;
        lastLinePosition = LINE_CENTER;
        enterState(T2_REACQUIRE_B_LINE);
      }
      break;

    case T2_REACQUIRE_B_LINE:
      driveForwardMagnitudes(SPEED_CAREFUL, SPEED_CAREFUL);

      if (lineReacquired()) {
        lastError = 0;
        enterState(T2_APPROACH_B_TAG);
      }

      if (millis() - stateStartMs > REACQUIRE_TIMEOUT_MS) {
        enterFailsafe("Could not reacquire B line after left turn");
      }
      break;

    case T2_APPROACH_B_TAG:
      followLinePD(SPEED_CAREFUL);

      if (bTagDetected) {
        latestUid.toCharArray(activeAirlockTagId, sizeof(activeAirlockTagId));
        lastServerRequestMs = 0;
        serverRequestStartedMs = 0;
        enterState(T2_REQUEST_AIRLOCK_B);
      }

      if (lineLostTooLong()) {
        enterFailsafe("Line lost while approaching B tag");
      }
      break;

    case T2_REQUEST_AIRLOCK_B:
      stopMotors();

      if (activeAirlockTagId[0] == '\0') {
        enterFailsafe("RFID B detected but tag id was empty");
        break;
      }

      if (lastServerRequestMs != 0 && millis() - lastServerRequestMs < SERVER_REQUEST_RETRY_MS) {
        break;
      }

      lastServerRequestMs = millis();

      if (requestOpenAirlock('B', activeAirlockTagId)) {
        serverRequestStartedMs = millis();
        lastServerRequestMs = serverRequestStartedMs;
        Serial.print("Airlock B request sent with tag ");
        Serial.println(activeAirlockTagId);
        enterState(T2_WAIT_AIRLOCK_B);
      } else {
        Serial.println("Airlock B request failed to send; retrying.");
      }
      break;

    case T2_WAIT_AIRLOCK_B:
      stopMotors();

      if (airlockRequestAccepted()) {
        Serial.println("Airlock B accepted. Continuing to first post-B junction.");
        enterState(T2_FOLLOW_AFTER_B_TAG_TO_JUNCTION);
        break;
      }

      if (airlockRequestRejected()) {
        Serial.println("Airlock B rejected. Retrying request.");
        enterState(T2_REQUEST_AIRLOCK_B);
        break;
      }

      if (millis() - serverRequestStartedMs >= SERVER_REPLY_TIMEOUT_MS &&
          millis() - lastServerRequestMs >= SERVER_REQUEST_RETRY_MS) {
        lastServerRequestMs = millis();
        serverRequestStartedMs = millis();
        Serial.println("Airlock B reply timeout; resending request.");
        requestOpenAirlock('B', activeAirlockTagId);
      }
      break;

    case T2_FOLLOW_AFTER_B_TAG_TO_JUNCTION:
      followLinePD(SPEED_CAREFUL);

      if (millis() - stateStartMs > EXIT_ROUTE_JUNCTION_IGNORE_MS &&
          baseJunctionStable()) {
        enterState(T2_TURN_LEFT_AFTER_B_TAG);
      }

      if (millis() - stateStartMs > EXIT_ROUTE_MAX_STRAIGHT_MS) {
        enterFailsafe("Could not find first junction after B tag");
      }

      if (lineLostTooLong()) {
        enterFailsafe("Line lost after B tag");
      }
      break;

    case T2_TURN_LEFT_AFTER_B_TAG:
      if (imuTurnPassedAngle(TURN_MIN_EARLY_STOP_DEG) &&
          validLineForEarlyTurnStop(TURN_IGNORE_LINE_MS)) {
        stopMotors();
        resetImuTurn();
        lastError = 0;
        baseJunctionFirstSeenMs = 0;
        enterState(T2_FOLLOW_TO_SECOND_JUNCTION);
        break;
      }

      if (runImuTurn(LEFT_TURN_ANGLE_DEG)) {
        lastError = 0;
        lastLinePosition = LINE_CENTER;
        enterState(T2_REACQUIRE_AFTER_B_TAG_LEFT);
      }
      break;

    case T2_REACQUIRE_AFTER_B_TAG_LEFT:
      driveForwardMagnitudes(SPEED_CAREFUL, SPEED_CAREFUL);

      if (lineReacquired()) {
        lastError = 0;
        baseJunctionFirstSeenMs = 0;
        enterState(T2_FOLLOW_TO_SECOND_JUNCTION);
      }

      if (millis() - stateStartMs > REACQUIRE_TIMEOUT_MS) {
        enterFailsafe("Could not reacquire line after first post-B left turn");
      }
      break;

    case T2_FOLLOW_TO_SECOND_JUNCTION:
      followLinePD(SPEED_CAREFUL);

      if (millis() - stateStartMs > EXIT_ROUTE_JUNCTION_IGNORE_MS &&
          baseJunctionStable()) {
        enterState(T2_TURN_RIGHT_AFTER_SECOND_JUNCTION);
      }

      if (millis() - stateStartMs > EXIT_ROUTE_MAX_STRAIGHT_MS) {
        enterFailsafe("Could not find second junction after B tag");
      }

      if (lineLostTooLong()) {
        enterFailsafe("Line lost before second junction after B tag");
      }
      break;

    case T2_TURN_RIGHT_AFTER_SECOND_JUNCTION:
      if (imuTurnPassedAngle(TURN_MIN_EARLY_STOP_DEG) &&
          validLineForEarlyTurnStop(TURN_IGNORE_LINE_MS)) {
        stopMotors();
        resetImuTurn();
        lastError = 0;
        enterState(T2_FINAL_FOLLOW_TEMP_STOP);
        break;
      }

      if (runImuTurn(RIGHT_TURN_ANGLE_DEG)) {
        lastError = 0;
        lastLinePosition = LINE_CENTER;
        enterState(T2_REACQUIRE_AFTER_SECOND_RIGHT);
      }
      break;

    case T2_REACQUIRE_AFTER_SECOND_RIGHT:
      driveForwardMagnitudes(SPEED_CAREFUL, SPEED_CAREFUL);

      if (lineReacquired()) {
        lastError = 0;
        enterState(T2_FINAL_FOLLOW_TEMP_STOP);
      }

      if (millis() - stateStartMs > REACQUIRE_TIMEOUT_MS) {
        enterFailsafe("Could not reacquire line after second post-B right turn");
      }
      break;

    case T2_FINAL_FOLLOW_TEMP_STOP:
      followLinePD(SPEED_CAREFUL);

      if (millis() - stateStartMs > FINAL_FOLLOW_TEMP_STOP_MS) {
        stopMotors();
        enterState(T2_DONE);
      }

      if (lineLostTooLong()) {
        enterFailsafe("Line lost during final temporary follow");
      }
      break;

    case T2_DONE:
      stopMotors();
      break;

    case T2_FAILSAFE_STOP:
      stopMotors();
      break;
  }
}

// --------------------
// Serial commands
// --------------------

void handleSerialCommands() {
  while (Serial.available()) {
    Serial.read();
  }
}

void startTask2FromButton() {
  task2Running = true;
  resetTask2Run();
  enterState(T2_LEAVE_DEPLOYMENT_AREA);
  Serial.println("BUTTON: Task 2 started.");
}

void stopTask2FromButton() {
  task2Running = false;
  stopMotors();
  resetImuTurn();
  enterState(T2_IDLE);
  Serial.println("BUTTON: Task 2 stopped.");
}

void handleControlButton() {
  bool reading = digitalRead(CONTROL_BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if (millis() - lastDebounceTime > DEBOUNCE_DELAY_MS) {
    if (reading != stableButtonState) {
      stableButtonState = reading;

      if (stableButtonState == CONTROL_BUTTON_PRESSED) {
        if (task2Running) {
          stopTask2FromButton();
        } else {
          startTask2FromButton();
        }
      }
    }
  }

  lastButtonReading = reading;
}

// --------------------
// Setup / loop
// --------------------

void setup() {
  Serial.begin(115200);

  unsigned long serialWaitStartMs = millis();
  while (!Serial && millis() - serialWaitStartMs < 1500) {
    delay(10);
  }

  delay(1000);

  Serial.println();
  Serial.println("Task 2: scripted base exit route + RFID-gated junction navigation");
  Serial.println("Mechanical control button: D8 to GND.");
  Serial.println("Press button once to start Task 2.");
  Serial.println("Press button again to stop motors.");
  Serial.println("Current mode: request Airlock B after RFID, then left at first junction, right at second, stop after 3s.");

  pinMode(CONTROL_BUTTON_PIN, INPUT_PULLUP);

  initTask2Communication();

  Wire.begin();
  Wire1.begin();
  Wire.setClock(400000);
  delay(500);

  mc.setBus(&Wire1);
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.setCommandTimeoutMilliseconds(1000);

  mc.setMaxAcceleration(LEFT_MOTOR_CHANNEL, 120);
  mc.setMaxDeceleration(LEFT_MOTOR_CHANNEL, 250);
  mc.setMaxAcceleration(RIGHT_MOTOR_CHANNEL, 120);
  mc.setMaxDeceleration(RIGHT_MOTOR_CHANNEL, 250);

  stopMotors();

  initImu();

  rfid.PCD_Init();
  delay(300);

  calibrateSensors();

  lastButtonReading = digitalRead(CONTROL_BUTTON_PIN);
  stableButtonState = lastButtonReading;

  Serial.println("Ready. Press the mechanical button to start Task 2.");
}

void loop() {
  mc.resetCommandTimeout();
  updateTask2Communication();
  handleControlButton();
  handleSerialCommands();
  runTask2();
  delay(5);
}
