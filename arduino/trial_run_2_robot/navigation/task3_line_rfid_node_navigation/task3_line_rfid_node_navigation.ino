#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include "ICM_20948.h"

// ============================================================
// Task 3: Line + RFID Node Navigation
// ============================================================
//
// Purpose:
// - Put the robot on any solid grid line.
// - It follows the black line.
// - RFID reader is ahead of the IR array, so RFID is detected first.
// - After RFID is detected, it slows down and waits for the IR array to
//   reach the node hole.
// - When the line is briefly lost after pending RFID, stop at the node.
// - Pause at the confirmed node.
// - Continue forward using IMU heading hold until the black line is found again.
// - Repeat.
//
// Runtime behaviour is non-blocking:
// - No delay() or while() is used for movement control.
// - updateTask3LineNavigation() advances the behaviour by one small step.
// - This structure is suitable for merging into a larger final state machine:
//     STATE_NAVIGATING_LINES -> updateTask3LineNavigation()
//
// Notes:
// - setup() calibration is still blocking in this standalone test sketch.
//   In final integration, calibration can happen once during boot/setup.

// --------------------
// Motoron setup
// --------------------

MotoronI2C mc(16);

const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;

const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;
const int FORWARD_SIGN = 1;

// --------------------
// Encoder setup
// --------------------

const int leftEncoderPinA = D2;
const int leftEncoderPinB = D3;

const int rightEncoderPinA = D4;
const int rightEncoderPinB = D5;

volatile long leftEncoderPos = 0;
volatile long rightEncoderPos = 0;

// --------------------
// IMU setup
// --------------------

ICM_20948_I2C myICM;

float yawDeg = 0.0;
float gyroBiasZ = 0.0;
float continueHeadingDeg = 0.0;

unsigned long lastImuUpdateMicros = 0;

const float GYRO_DEADBAND_DPS = 0.10;

// --------------------
// RFID setup
// --------------------

#define RFID_ADDR 0x28
MFRC522_I2C rfid(RFID_ADDR, -1, &Wire1);

uint32_t latestTagId = 0;
bool tagDetected = false;
uint32_t nodeCandidateTagId = 0;
bool nodeCandidateTagSeen = false;
uint32_t pendingNodeTagId = 0;
bool pendingNodeTag = false;
unsigned long pendingNodeTagSeenMs = 0;

// --------------------
// Task 3 tuning
// --------------------

const int GRID_LINE_SPEED = 220;
const int AFTER_RFID_SLOW_SPEED = 150;
const int REACQUIRE_SPEED = 150;
const int MAX_SPEED = 360;

float lineKp = 0.040;
float lineKd = 0.10;

// Heading hold while crossing the hole and finding the line again.
float headingKp = 4.0;
const int HEADING_MAX_CORRECTION = 70;

// Do not treat an immediate startup line-loss as a node.
// Tune this down if the first node is very close to the start position.
const long MIN_COUNTS_BEFORE_NODE_CANDIDATE = 250;

// RFID must be seen while the robot is stopped over the node candidate.
const unsigned long NODE_RFID_CONFIRM_TIMEOUT_MS = 1200;

// RFID reader is physically ahead of the IR array.
// After RFID is detected, the robot keeps following the line slowly until
// the IR array reaches the node hole and briefly loses the black line.
const unsigned long RFID_TO_HOLE_TIMEOUT_MS = 1800;

// Pause at every confirmed node before moving to the next segment.
const unsigned long NODE_PAUSE_MS = 600;

// Time allowed to cross the hole and find the black line again.
const unsigned long REACQUIRE_AFTER_NODE_TIMEOUT_MS = 1800;

// How many confirmed RFID nodes to visit in this standalone test.
// For final integration, this can come from STATE_PLAN.
const int TARGET_CONFIRMED_NODES = 4;

// Global timeout for this standalone Task 3 test.
const unsigned long TASK3_TIMEOUT_MS = 45000;

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
const uint16_t LINE_TOTAL_THRESHOLD = 300;

uint16_t lastLinePosition = LINE_CENTER;
int lastError = 0;
int activeSensorCount = 0;
bool lineDetected = false;

// --------------------
// Task 3 state machine
// --------------------

enum Task3LineState {
  T3_LINE_IDLE,
  T3_LINE_FOLLOW,
  T3_NODE_CANDIDATE,
  T3_NODE_CONFIRMED_PAUSE,
  T3_REACQUIRE_AFTER_NODE,
  T3_LINE_DONE,
  T3_LINE_FAILSAFE
};

Task3LineState task3LineState = T3_LINE_IDLE;

int confirmedNodes = 0;

unsigned long taskStartMs = 0;
unsigned long stateStartMs = 0;
unsigned long nodeCandidateStartMs = 0;
unsigned long nodePauseStartMs = 0;
unsigned long reacquireStartMs = 0;
unsigned long lastDebugPrintMs = 0;

// --------------------
// Forward declarations
// --------------------

void enterTask3State(Task3LineState nextState);
void enterNodeCandidate();
void enterReacquireAfterNode();
void enterTask3Failsafe(const char* reason);
void rememberPendingNodeTag(uint32_t tagId);
void clearPendingNodeTag();
void confirmPendingNodeAtHole();

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

// --------------------
// Encoder helpers
// --------------------

void readLeftEncoder() {
  if (digitalRead(leftEncoderPinA) == digitalRead(leftEncoderPinB)) {
    leftEncoderPos--;
  } else {
    leftEncoderPos++;
  }
}

void readRightEncoder() {
  if (digitalRead(rightEncoderPinA) == digitalRead(rightEncoderPinB)) {
    rightEncoderPos--;
  } else {
    rightEncoderPos++;
  }
}

void resetSegmentEncoders() {
  noInterrupts();
  leftEncoderPos = 0;
  rightEncoderPos = 0;
  interrupts();
}

long getAverageEncoderCounts() {
  noInterrupts();
  long leftCounts = leftEncoderPos;
  long rightCounts = rightEncoderPos;
  interrupts();

  leftCounts = abs(leftCounts);
  rightCounts = abs(rightCounts);

  return (leftCounts + rightCounts) / 2;
}

bool nodeCandidateAllowed() {
  return getAverageEncoderCounts() >= MIN_COUNTS_BEFORE_NODE_CANDIDATE;
}

// --------------------
// UI helpers
// --------------------

void printCountdown(const char* message, int seconds) {
  Serial.println(message);

  for (int remaining = seconds; remaining > 0; remaining--) {
    Serial.print("Starting in ");
    Serial.print(remaining);
    Serial.println("...");
    delay(1000);
  }
}

// --------------------
// IMU helpers
// --------------------

float normalizeAngleDeg(float angle) {
  while (angle > 180.0) {
    angle -= 360.0;
  }

  while (angle < -180.0) {
    angle += 360.0;
  }

  return angle;
}

float headingErrorDeg(float targetDeg, float currentDeg) {
  return normalizeAngleDeg(targetDeg - currentDeg);
}

bool initImu() {
  Serial.println();
  Serial.println("=== IMU INIT ===");

  myICM.begin(Wire, 1);

  if (myICM.status != ICM_20948_Stat_Ok) {
    Serial.print("IMU init failed. status=");
    Serial.println(myICM.statusString());
    return false;
  }

  Serial.println("IMU connected.");
  return true;
}

void calibrateGyroZ() {
  Serial.println();
  Serial.println("=== IMU GYRO Z CALIBRATION ===");
  printCountdown("Keep the robot completely still.", 3);

  const int samples = 500;
  float sum = 0.0;

  for (int i = 0; i < samples; i++) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      sum += myICM.gyrZ();
    }
    delay(5);
  }

  gyroBiasZ = sum / samples;
  yawDeg = 0.0;
  continueHeadingDeg = 0.0;
  lastImuUpdateMicros = micros();

  Serial.print("gyroBiasZ=");
  Serial.println(gyroBiasZ, 4);
  Serial.println("IMU calibration complete.");
}

void updateImuYaw() {
  if (!myICM.dataReady()) {
    return;
  }

  unsigned long nowMicros = micros();

  if (lastImuUpdateMicros == 0) {
    lastImuUpdateMicros = nowMicros;
  }

  float dt = (nowMicros - lastImuUpdateMicros) / 1000000.0;
  lastImuUpdateMicros = nowMicros;

  myICM.getAGMT();

  float gyroZ = myICM.gyrZ() - gyroBiasZ;

  if (abs(gyroZ) < GYRO_DEADBAND_DPS) {
    gyroZ = 0.0;
  }

  yawDeg = normalizeAngleDeg(yawDeg + gyroZ * dt);
}

// --------------------
// RFID reading
// --------------------

void updateRfid() {
  tagDetected = false;

  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return;
  }

  latestTagId = 0;

  for (byte i = 0; i < rfid.uid.size && i < 4; i++) {
    latestTagId = (latestTagId << 8) | rfid.uid.uidByte[i];
  }

  tagDetected = true;

  Serial.print("RFID node detected: 0x");
  Serial.println(latestTagId, HEX);

  if (task3LineState == T3_LINE_FOLLOW) {
    rememberPendingNodeTag(latestTagId);
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
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
  Serial.println("=== TASK 3 GRID LINE CALIBRATION ===");
  printCountdown("Move the sensor array over white floor, black grid line, and node holes.", 3);
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

  if (!lineDetected) {
    return lastLinePosition;
  }

  lastLinePosition = weightedSum / total;
  return lastLinePosition;
}

void updateLineSensors() {
  readQTR_RC();
  calculateCalibratedValues();
  readLinePosition();
}

// --------------------
// Movement behaviours
// --------------------

void followGridLinePD(int baseSpeed) {
  int error = (int)lastLinePosition - LINE_CENTER;
  int derivative = error - lastError;
  int correction = (int)(lineKp * error + lineKd * derivative);

  int leftMagnitude = baseSpeed + correction;
  int rightMagnitude = baseSpeed - correction;

  driveForwardMagnitudes(leftMagnitude, rightMagnitude);
  lastError = error;
}

void driveForwardWithHeadingHold(float targetHeadingDeg, int baseSpeed) {
  float error = headingErrorDeg(targetHeadingDeg, yawDeg);
  int correction = (int)(headingKp * error);
  correction = constrain(correction, -HEADING_MAX_CORRECTION, HEADING_MAX_CORRECTION);

  int leftMagnitude = baseSpeed - correction;
  int rightMagnitude = baseSpeed + correction;

  driveForwardMagnitudes(leftMagnitude, rightMagnitude);
}

// --------------------
// Task 3 navigation API
// --------------------

void startTask3LineNavigation() {
  confirmedNodes = 0;
  taskStartMs = millis();
  nodeCandidateStartMs = 0;
  nodePauseStartMs = 0;
  reacquireStartMs = 0;
  latestTagId = 0;
  tagDetected = false;
  nodeCandidateTagId = 0;
  nodeCandidateTagSeen = false;
  clearPendingNodeTag();
  lastError = 0;
  lastLinePosition = LINE_CENTER;
  continueHeadingDeg = yawDeg;
  resetSegmentEncoders();
  enterTask3State(T3_LINE_FOLLOW);
}

void stopTask3LineNavigation() {
  stopMotors();
  enterTask3State(T3_LINE_IDLE);
}

bool task3LineNavigationComplete() {
  return task3LineState == T3_LINE_DONE;
}

bool task3LineNavigationFailed() {
  return task3LineState == T3_LINE_FAILSAFE;
}

// Call this once per loop while currentState == STATE_NAVIGATING_LINES.
void updateTask3LineNavigation() {
  updateImuYaw();
  updateLineSensors();
  updateRfid();

  if (taskStartMs > 0 &&
      millis() - taskStartMs > TASK3_TIMEOUT_MS &&
      task3LineState != T3_LINE_IDLE &&
      task3LineState != T3_LINE_DONE &&
      task3LineState != T3_LINE_FAILSAFE) {
    enterTask3Failsafe("Task 3 line navigation timeout");
    return;
  }

  switch (task3LineState) {
    case T3_LINE_IDLE:
      stopMotors();
      break;

    case T3_LINE_FOLLOW:
      if (pendingNodeTag &&
          millis() - pendingNodeTagSeenMs > RFID_TO_HOLE_TIMEOUT_MS) {
        Serial.println("Pending RFID expired before IR reached hole.");
        clearPendingNodeTag();
      }

      if (!lineDetected) {
        if (pendingNodeTag && nodeCandidateAllowed()) {
          confirmPendingNodeAtHole();
        } else if (nodeCandidateAllowed()) {
          enterNodeCandidate();
        } else {
          enterTask3Failsafe("Line lost before minimum node distance");
        }
        break;
      }

      followGridLinePD(pendingNodeTag ? AFTER_RFID_SLOW_SPEED : GRID_LINE_SPEED);
      break;

    case T3_NODE_CANDIDATE:
      stopMotors();

      if (tagDetected) {
        nodeCandidateTagId = latestTagId;
        nodeCandidateTagSeen = true;
      }

      if (nodeCandidateTagSeen) {
        confirmedNodes++;
        nodePauseStartMs = millis();

        Serial.print("Confirmed RFID node. confirmedNodes=");
        Serial.print(confirmedNodes);
        Serial.print("/");
        Serial.print(TARGET_CONFIRMED_NODES);
        Serial.print(" uid=0x");
        Serial.println(nodeCandidateTagId, HEX);

        enterTask3State(T3_NODE_CONFIRMED_PAUSE);
        break;
      }

      if (lineDetected) {
        Serial.println("Node candidate rejected: line returned before RFID.");
        lastError = 0;
        enterTask3State(T3_LINE_FOLLOW);
        break;
      }

      if (millis() - nodeCandidateStartMs > NODE_RFID_CONFIRM_TIMEOUT_MS) {
        enterTask3Failsafe("Line lost like a node, but RFID was not detected");
      }
      break;

    case T3_NODE_CONFIRMED_PAUSE:
      stopMotors();

      if (millis() - nodePauseStartMs >= NODE_PAUSE_MS) {
        if (confirmedNodes >= TARGET_CONFIRMED_NODES) {
          enterTask3State(T3_LINE_DONE);
          break;
        }

        enterReacquireAfterNode();
      }
      break;

    case T3_REACQUIRE_AFTER_NODE:
      if (lineDetected) {
        Serial.println("Line reacquired after node.");
        lastError = 0;
        lastLinePosition = LINE_CENTER;
        enterTask3State(T3_LINE_FOLLOW);
        break;
      }

      driveForwardWithHeadingHold(continueHeadingDeg, REACQUIRE_SPEED);

      if (millis() - reacquireStartMs > REACQUIRE_AFTER_NODE_TIMEOUT_MS) {
        enterTask3Failsafe("Could not reacquire line after RFID node");
      }
      break;

    case T3_LINE_DONE:
      stopMotors();
      break;

    case T3_LINE_FAILSAFE:
      stopMotors();
      break;
  }
}

// --------------------
// State helpers
// --------------------

void enterTask3State(Task3LineState nextState) {
  task3LineState = nextState;
  stateStartMs = millis();

  Serial.print("Task 3 line state -> ");
  Serial.println((int)task3LineState);
}

void enterNodeCandidate() {
  stopMotors();
  nodeCandidateStartMs = millis();
  continueHeadingDeg = yawDeg;
  nodeCandidateTagId = latestTagId;
  nodeCandidateTagSeen = tagDetected;

  Serial.print("Node candidate: line lost after avgCounts=");
  Serial.println(getAverageEncoderCounts());

  enterTask3State(T3_NODE_CANDIDATE);
}

void enterReacquireAfterNode() {
  resetSegmentEncoders();
  reacquireStartMs = millis();
  lastError = 0;
  lastLinePosition = LINE_CENTER;
  latestTagId = 0;
  tagDetected = false;
  nodeCandidateTagId = 0;
  nodeCandidateTagSeen = false;
  clearPendingNodeTag();

  Serial.print("Leaving node. Holding heading=");
  Serial.println(continueHeadingDeg, 1);

  enterTask3State(T3_REACQUIRE_AFTER_NODE);
}

void enterTask3Failsafe(const char* reason) {
  stopMotors();
  Serial.print("FAILSAFE: ");
  Serial.println(reason);
  enterTask3State(T3_LINE_FAILSAFE);
}

void rememberPendingNodeTag(uint32_t tagId) {
  pendingNodeTagId = tagId;
  pendingNodeTag = true;
  pendingNodeTagSeenMs = millis();

  Serial.print("Pending RFID node. Waiting for IR hole. uid=0x");
  Serial.println(pendingNodeTagId, HEX);
}

void clearPendingNodeTag() {
  pendingNodeTagId = 0;
  pendingNodeTag = false;
  pendingNodeTagSeenMs = 0;
}

void confirmPendingNodeAtHole() {
  stopMotors();
  confirmedNodes++;
  nodePauseStartMs = millis();
  nodeCandidateTagId = pendingNodeTagId;
  nodeCandidateTagSeen = true;
  continueHeadingDeg = yawDeg;

  Serial.print("Confirmed RFID node at IR hole. confirmedNodes=");
  Serial.print(confirmedNodes);
  Serial.print("/");
  Serial.print(TARGET_CONFIRMED_NODES);
  Serial.print(" uid=0x");
  Serial.println(nodeCandidateTagId, HEX);

  clearPendingNodeTag();
  enterTask3State(T3_NODE_CONFIRMED_PAUSE);
}

// --------------------
// Debug printing
// --------------------

void printDebugStatus() {
  if (millis() - lastDebugPrintMs < 200) {
    return;
  }

  lastDebugPrintMs = millis();

  Serial.print("state=");
  Serial.print((int)task3LineState);
  Serial.print(" nodes=");
  Serial.print(confirmedNodes);
  Serial.print("/");
  Serial.print(TARGET_CONFIRMED_NODES);
  Serial.print(" line=");
  Serial.print(lineDetected ? 1 : 0);
  Serial.print(" active=");
  Serial.print(activeSensorCount);
  Serial.print(" error=");
  Serial.print(lastError);
  Serial.print(" avg=");
  Serial.print(getAverageEncoderCounts());
  Serial.print(" yaw=");
  Serial.print(yawDeg, 1);
  Serial.print(" hold=");
  Serial.print(continueHeadingDeg, 1);
  Serial.print(" tag=");
  Serial.print(tagDetected ? 1 : 0);
  Serial.print(" pending=");
  Serial.print(pendingNodeTag ? 1 : 0);
  Serial.print(" uid=0x");
  Serial.println(latestTagId, HEX);
}

// --------------------
// Serial commands for standalone test
// --------------------

void handleSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  char command = Serial.read();

  if (command == 'g' || command == 'G') {
    startTask3LineNavigation();
  }

  if (command == 's' || command == 'S') {
    stopTask3LineNavigation();
  }
}

// --------------------
// Setup / loop
// --------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  delay(1000);

  Serial.println();
  Serial.println("Task 3: line + RFID node navigation");
  Serial.println("Commands: g = start, s = stop");
  Serial.println("Logic: follow line -> line lost means node candidate -> RFID confirms node -> heading-hold reacquire.");

  Wire.begin();
  Wire.setClock(400000);

  Wire1.begin();
  Wire1.setClock(400000);
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

  pinMode(leftEncoderPinA, INPUT_PULLUP);
  pinMode(leftEncoderPinB, INPUT_PULLUP);
  pinMode(rightEncoderPinA, INPUT_PULLUP);
  pinMode(rightEncoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), readLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), readRightEncoder, CHANGE);

  stopMotors();

  if (!initImu()) {
    Serial.println("IMU failed. This sketch needs IMU heading hold to leave node holes.");
  } else {
    calibrateGyroZ();
  }

  rfid.PCD_Init();
  delay(300);
  Serial.println("RFID reader ready.");

  calibrateSensors();

  Serial.println("Ready. Place robot on one grid line and type g.");
}

void loop() {
  mc.resetCommandTimeout();
  handleSerialCommands();
  updateTask3LineNavigation();
  printDebugStatus();
  delay(5);
}
