#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include "ICM_20948.h"

// ============================================================
// Task 4: Open-Field Hole Navigation
// ============================================================
//
// Purpose:
// - Navigate on the right half of the arena where there are no black lines.
// - Move one grid segment at a time between node holes.
// - Use encoder counts for distance and IMU yaw for heading hold.
// - Near the expected node, use IR hole detection to stop over the node.
// - Optionally use RFID to confirm the node identity.
//
// Runtime behaviour is non-blocking:
// - updateTask4OpenNavigation() advances the behaviour by one small step.
// - No blocking while-loops or delay() are used during movement control.
// - This structure is intended to merge into final Robot_Main as:
//     STATE_NAVIGATING_OPEN -> updateTask4OpenNavigation()
//
// Notes:
// - setup() IMU calibration is blocking in this standalone test sketch.
//   In final integration, calibration should happen once during boot/setup.

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

float yaw = 0.0;
float gyroBiasZ = 0.0;
float targetHeadingDeg = 0.0;
unsigned long lastImuUpdateMicros = 0;

const float GYRO_DEADBAND_DPS = 0.1;

// --------------------
// RFID setup
// --------------------

#define RFID_ADDR 0x28
MFRC522_I2C rfid(RFID_ADDR, -1, &Wire1);

uint32_t latestTagId = 0;
bool tagDetected = false;
unsigned long lastTagSeenMs = 0;

// --------------------
// IR hole detection setup
// --------------------

const uint8_t SensorCount = 9;
const uint8_t sensorPins[SensorCount] = {
  22, 23, 24, 25, 26, 27, 28, 29, 30
};

uint16_t sensorValues[SensorCount];
uint16_t woodBaselines[SensorCount];

const uint16_t IR_TIMEOUT_MICROS = 2500;

int emptyHoleCount = 0;
bool holeDetected = false;
float holeCenter = 4.0;
float holeError = 0.0;
int holeStableHits = 0;

// --------------------
// Task 4 tuning
// --------------------

const int OPEN_FIELD_SPEED = 140;
const int OPEN_FIELD_SLOW_SPEED = 90;
const int OPEN_SEARCH_SPEED = 80;
const int OPEN_ALIGN_SPEED = 90;
const int MAX_SPEED = 360;

// TODO: Measure this on the real grid.
// This should be encoder counts from one node/hole to the next adjacent node/hole.
const long OPEN_GRID_SEGMENT_COUNTS = 1200;

// How many open-field grid segments to travel in this standalone test.
// For final integration, this should come from STATE_PLAN.
const int OPEN_SEGMENTS_TO_RUN = 3;

const long OPEN_SLOWDOWN_COUNTS = 250;
const long OPEN_NODE_SEARCH_WINDOW_COUNTS = 350;

// Heading-hold controller. Start with P-only.
const float OPEN_HEADING_KP = 4.0;
const int OPEN_HEADING_MAX_CORRECTION = 80;

const unsigned long OPEN_NODE_PAUSE_MS = 700;
const unsigned long OPEN_SEGMENT_TIMEOUT_MS = 6000;
const unsigned long OPEN_NODE_SEARCH_TIMEOUT_MS = 3000;
const unsigned long TASK4_TIMEOUT_MS = 45000;

// Start with IR-only confirmation for testing. Set true when RFID placement
// and read range are reliable enough for final node identity checks.
const bool REQUIRE_RFID_CONFIRMATION = false;

const uint8_t HOLE_EMPTY_MIN_COUNT = 2;
const float HOLE_CENTER_TARGET = 4.0;
const float HOLE_CENTER_TOLERANCE = 0.35;
const int HOLE_CENTER_REQUIRED_HITS = 5;

const uint16_t HOLE_WOOD_OFFSET = 25;
const uint16_t HOLE_EMPTY_OFFSET = 55;
const uint16_t HOLE_SEED_OFFSET = 130;

// --------------------
// Task 4 state machine
// --------------------

enum Task4OpenState {
  T4_OPEN_IDLE,
  T4_OPEN_DRIVE_SEGMENT,
  T4_OPEN_SEARCH_NODE,
  T4_OPEN_ALIGN_HOLE,
  T4_OPEN_CONFIRM_NODE,
  T4_OPEN_NODE_PAUSE,
  T4_OPEN_DONE,
  T4_OPEN_FAILSAFE
};

Task4OpenState task4OpenState = T4_OPEN_IDLE;

int completedOpenSegments = 0;

unsigned long taskStartMs = 0;
unsigned long stateStartMs = 0;
unsigned long segmentStartMs = 0;
unsigned long nodeSearchStartMs = 0;
unsigned long nodePauseStartMs = 0;
unsigned long lastDebugPrintMs = 0;

// --------------------
// Forward declarations
// --------------------

void enterTask4State(Task4OpenState nextState);
void enterTask4Failsafe(const char* reason);
void enterTask4NodePause();

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

bool currentOpenSegmentReached() {
  return getAverageEncoderCounts() >= OPEN_GRID_SEGMENT_COUNTS;
}

bool nearExpectedOpenNode() {
  long searchStartCounts = OPEN_GRID_SEGMENT_COUNTS - OPEN_NODE_SEARCH_WINDOW_COUNTS;

  if (searchStartCounts < 0) {
    searchStartCounts = 0;
  }

  return getAverageEncoderCounts() >= searchStartCounts;
}

int currentOpenSegmentSpeed() {
  long counts = getAverageEncoderCounts();
  long remainingCounts = OPEN_GRID_SEGMENT_COUNTS - counts;

  if (remainingCounts > 0 && remainingCounts < OPEN_SLOWDOWN_COUNTS) {
    return OPEN_FIELD_SLOW_SPEED;
  }

  return OPEN_FIELD_SPEED;
}

// --------------------
// IMU helpers
// --------------------

float headingErrorDeg(float targetDeg, float currentDeg) {
  float error = targetDeg - currentDeg;

  while (error > 180.0) {
    error -= 360.0;
  }

  while (error < -180.0) {
    error += 360.0;
  }

  return error;
}

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

  float dt = (currentMicros - lastImuUpdateMicros) / 1000000.0;
  lastImuUpdateMicros = currentMicros;

  if (dt <= 0.0 || dt > 0.2) {
    return;
  }

  float gyroZ = myICM.gyrZ() - gyroBiasZ;

  if (abs(gyroZ) > GYRO_DEADBAND_DPS) {
    yaw += gyroZ * dt;
  }
}

void calibrateGyroZ() {
  Serial.println();
  Serial.println("=== TASK 4 IMU CALIBRATION ===");
  Serial.println("Keep the robot completely still.");

  for (int remaining = 3; remaining > 0; remaining--) {
    Serial.print("Starting in ");
    Serial.print(remaining);
    Serial.println("...");
    delay(1000);
  }

  gyroBiasZ = 0.0;
  int samples = 0;

  while (samples < 1000) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      gyroBiasZ += myICM.gyrZ();
      samples++;

      if (samples % 100 == 0) {
        Serial.print("IMU samples: ");
        Serial.print(samples);
        Serial.println(" / 1000");
      }
    }

    delay(5);
  }

  gyroBiasZ /= 1000.0;
  yaw = 0.0;
  lastImuUpdateMicros = micros();

  Serial.print("Gyro Z bias = ");
  Serial.println(gyroBiasZ, 4);
  Serial.println("IMU calibration complete.");
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

// --------------------
// IR hole detection helpers
// --------------------

void readQTR_RC() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }

  delayMicroseconds(10);

  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorValues[i] = IR_TIMEOUT_MICROS;
  }

  unsigned long startTime = micros();
  bool allDone = false;

  while (!allDone && (micros() - startTime < IR_TIMEOUT_MICROS)) {
    allDone = true;

    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] == IR_TIMEOUT_MICROS) {
        if (digitalRead(sensorPins[i]) == LOW) {
          sensorValues[i] = micros() - startTime;
        } else {
          allDone = false;
        }
      }
    }
  }
}

void calibrateWoodBaseline() {
  Serial.println();
  Serial.println("=== TASK 4 IR WOOD BASELINE CALIBRATION ===");
  Serial.println("Place all IR sensors over normal wood/open-field floor.");

  for (int remaining = 3; remaining > 0; remaining--) {
    Serial.print("Starting in ");
    Serial.print(remaining);
    Serial.println("...");
    delay(1000);
  }

  uint32_t sums[SensorCount] = {0};

  for (int sample = 0; sample < 30; sample++) {
    readQTR_RC();

    for (uint8_t i = 0; i < SensorCount; i++) {
      sums[i] += sensorValues[i];
    }

    delay(20);
  }

  Serial.println("--- IR wood baselines ---");

  for (uint8_t i = 0; i < SensorCount; i++) {
    woodBaselines[i] = sums[i] / 30;

    Serial.print("sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(woodBaselines[i]);
  }
}

uint8_t getSurfaceState(uint8_t sensorIndex, uint16_t rawValue) {
  uint16_t woodMax = woodBaselines[sensorIndex] + HOLE_WOOD_OFFSET;
  uint16_t emptyMax = woodBaselines[sensorIndex] + HOLE_EMPTY_OFFSET;
  uint16_t seedMax = woodBaselines[sensorIndex] + HOLE_SEED_OFFSET;

  if (rawValue < woodMax) {
    return 0;
  }

  if (rawValue < emptyMax) {
    return 2;
  }

  if (rawValue < seedMax) {
    return 3;
  }

  return 1;
}

void updateHoleDetection() {
  readQTR_RC();

  emptyHoleCount = 0;
  int leftBound = -1;
  int rightBound = -1;

  for (uint8_t i = 0; i < SensorCount; i++) {
    uint8_t state = getSurfaceState(i, sensorValues[i]);

    if (state == 2) {
      emptyHoleCount++;

      if (leftBound == -1) {
        leftBound = i;
      }

      rightBound = i;
    }
  }

  holeDetected = emptyHoleCount >= HOLE_EMPTY_MIN_COUNT;

  if (holeDetected) {
    holeCenter = (leftBound + rightBound) / 2.0;
    holeError = HOLE_CENTER_TARGET - holeCenter;
  } else {
    holeCenter = -1.0;
    holeError = 0.0;
  }
}

// --------------------
// RFID helpers
// --------------------

void updateRfid() {
  tagDetected = false;

  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return;
  }

  uint32_t packedId = 0;

  for (byte i = 0; i < rfid.uid.size && i < 4; i++) {
    packedId = (packedId << 8) | rfid.uid.uidByte[i];
  }

  latestTagId = packedId;
  tagDetected = true;
  lastTagSeenMs = millis();

  Serial.print("RFID node tag: 0x");
  Serial.println(latestTagId, HEX);

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// --------------------
// Open-field movement
// --------------------

void driveOpenFieldWithHeadingHold() {
  int baseSpeed = currentOpenSegmentSpeed();
  float error = headingErrorDeg(targetHeadingDeg, yaw);

  int correction = (int)(OPEN_HEADING_KP * error);
  correction = constrain(correction, -OPEN_HEADING_MAX_CORRECTION, OPEN_HEADING_MAX_CORRECTION);

  int leftMagnitude = baseSpeed + correction;
  int rightMagnitude = baseSpeed - correction;

  driveForwardMagnitudes(leftMagnitude, rightMagnitude);
}

void driveOpenFieldSearch() {
  float error = headingErrorDeg(targetHeadingDeg, yaw);

  int correction = (int)(OPEN_HEADING_KP * error);
  correction = constrain(correction, -OPEN_HEADING_MAX_CORRECTION, OPEN_HEADING_MAX_CORRECTION);

  int leftMagnitude = OPEN_SEARCH_SPEED + correction;
  int rightMagnitude = OPEN_SEARCH_SPEED - correction;

  driveForwardMagnitudes(leftMagnitude, rightMagnitude);
}

void rotateToCenterHole() {
  if (holeError > 0) {
    driveMotors(-OPEN_ALIGN_SPEED, OPEN_ALIGN_SPEED);
  } else {
    driveMotors(OPEN_ALIGN_SPEED, -OPEN_ALIGN_SPEED);
  }
}

// --------------------
// Task 4 navigation API
// --------------------

void startTask4OpenNavigation() {
  completedOpenSegments = 0;
  taskStartMs = millis();
  segmentStartMs = millis();
  nodeSearchStartMs = 0;
  nodePauseStartMs = 0;
  latestTagId = 0;
  tagDetected = false;
  holeStableHits = 0;
  targetHeadingDeg = yaw;
  resetSegmentEncoders();
  enterTask4State(T4_OPEN_DRIVE_SEGMENT);
}

void stopTask4OpenNavigation() {
  stopMotors();
  enterTask4State(T4_OPEN_IDLE);
}

bool task4OpenNavigationComplete() {
  return task4OpenState == T4_OPEN_DONE;
}

bool task4OpenNavigationFailed() {
  return task4OpenState == T4_OPEN_FAILSAFE;
}

// Call this once per loop while currentState == STATE_NAVIGATING_OPEN.
void updateTask4OpenNavigation() {
  updateImuYaw();
  updateRfid();
  updateHoleDetection();

  if (taskStartMs > 0 &&
      millis() - taskStartMs > TASK4_TIMEOUT_MS &&
      task4OpenState != T4_OPEN_IDLE &&
      task4OpenState != T4_OPEN_DONE &&
      task4OpenState != T4_OPEN_FAILSAFE) {
    enterTask4Failsafe("Task 4 open navigation timeout");
    return;
  }

  switch (task4OpenState) {
    case T4_OPEN_IDLE:
      stopMotors();
      break;

    case T4_OPEN_DRIVE_SEGMENT:
      if (nearExpectedOpenNode()) {
        nodeSearchStartMs = millis();
        holeStableHits = 0;
        Serial.println("Near expected open-field node. Searching for hole.");
        enterTask4State(T4_OPEN_SEARCH_NODE);
        break;
      }

      if (millis() - segmentStartMs > OPEN_SEGMENT_TIMEOUT_MS) {
        enterTask4Failsafe("Open-field segment timeout");
        break;
      }

      driveOpenFieldWithHeadingHold();
      break;

    case T4_OPEN_SEARCH_NODE:
      if (holeDetected) {
        stopMotors();
        holeStableHits = 0;
        Serial.println("Open-field hole detected. Aligning.");
        enterTask4State(T4_OPEN_ALIGN_HOLE);
        break;
      }

      if (currentOpenSegmentReached() &&
          millis() - nodeSearchStartMs > OPEN_NODE_SEARCH_TIMEOUT_MS) {
        enterTask4Failsafe("Could not find open-field hole near expected node");
        break;
      }

      if (millis() - segmentStartMs > OPEN_SEGMENT_TIMEOUT_MS) {
        enterTask4Failsafe("Open-field search timeout");
        break;
      }

      driveOpenFieldSearch();
      break;

    case T4_OPEN_ALIGN_HOLE:
      if (!holeDetected) {
        holeStableHits = 0;
        nodeSearchStartMs = millis();
        enterTask4State(T4_OPEN_SEARCH_NODE);
        break;
      }

      if (holeError >= -HOLE_CENTER_TOLERANCE &&
          holeError <= HOLE_CENTER_TOLERANCE) {
        stopMotors();
        holeStableHits++;

        if (holeStableHits >= HOLE_CENTER_REQUIRED_HITS) {
          enterTask4State(T4_OPEN_CONFIRM_NODE);
        }
        break;
      }

      holeStableHits = 0;
      rotateToCenterHole();
      break;

    case T4_OPEN_CONFIRM_NODE:
      stopMotors();

      if (REQUIRE_RFID_CONFIRMATION && latestTagId == 0) {
        if (millis() - stateStartMs > OPEN_NODE_SEARCH_TIMEOUT_MS) {
          enterTask4Failsafe("Hole centered but RFID did not confirm node");
        }
        break;
      }

      completedOpenSegments++;

      Serial.print("Confirmed open-field node. completedSegments=");
      Serial.print(completedOpenSegments);
      Serial.print("/");
      Serial.print(OPEN_SEGMENTS_TO_RUN);
      Serial.print(" uid=0x");
      Serial.println(latestTagId, HEX);

      enterTask4NodePause();
      break;

    case T4_OPEN_NODE_PAUSE:
      stopMotors();

      if (millis() - nodePauseStartMs >= OPEN_NODE_PAUSE_MS) {
        if (completedOpenSegments >= OPEN_SEGMENTS_TO_RUN) {
          enterTask4State(T4_OPEN_DONE);
          break;
        }

        targetHeadingDeg = yaw;
        latestTagId = 0;
        tagDetected = false;
        holeStableHits = 0;
        resetSegmentEncoders();
        segmentStartMs = millis();
        enterTask4State(T4_OPEN_DRIVE_SEGMENT);
      }
      break;

    case T4_OPEN_DONE:
      stopMotors();
      break;

    case T4_OPEN_FAILSAFE:
      stopMotors();
      break;
  }
}

// --------------------
// State helpers
// --------------------

void enterTask4State(Task4OpenState nextState) {
  task4OpenState = nextState;
  stateStartMs = millis();

  Serial.print("Task 4 open state -> ");
  Serial.println((int)task4OpenState);
}

void enterTask4NodePause() {
  stopMotors();
  nodePauseStartMs = millis();
  enterTask4State(T4_OPEN_NODE_PAUSE);
}

void enterTask4Failsafe(const char* reason) {
  stopMotors();
  Serial.print("FAILSAFE: ");
  Serial.println(reason);
  enterTask4State(T4_OPEN_FAILSAFE);
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
  Serial.print((int)task4OpenState);
  Serial.print(" seg=");
  Serial.print(completedOpenSegments);
  Serial.print("/");
  Serial.print(OPEN_SEGMENTS_TO_RUN);
  Serial.print(" yaw=");
  Serial.print(yaw, 1);
  Serial.print(" targetYaw=");
  Serial.print(targetHeadingDeg, 1);
  Serial.print(" err=");
  Serial.print(headingErrorDeg(targetHeadingDeg, yaw), 1);
  Serial.print(" Lenc=");
  Serial.print(leftEncoderPos);
  Serial.print(" Renc=");
  Serial.print(rightEncoderPos);
  Serial.print(" avg=");
  Serial.print(getAverageEncoderCounts());
  Serial.print(" targetSeg=");
  Serial.print(OPEN_GRID_SEGMENT_COUNTS);
  Serial.print(" hole=");
  Serial.print(holeDetected ? 1 : 0);
  Serial.print(" empty=");
  Serial.print(emptyHoleCount);
  Serial.print(" hCenter=");
  Serial.print(holeCenter, 1);
  Serial.print(" hErr=");
  Serial.print(holeError, 1);
  Serial.print(" tag=");
  if (latestTagId == 0) {
    Serial.println("none");
  } else {
    Serial.println(latestTagId, HEX);
  }
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
    startTask4OpenNavigation();
  }

  if (command == 's' || command == 'S') {
    stopTask4OpenNavigation();
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
  Serial.println("Task 4: open-field hole navigation");
  Serial.println("Commands: g = start, s = stop");
  Serial.println("This version uses IMU heading hold + encoder distance + IR hole detection.");

  Wire.begin();
  Wire1.begin();
  Wire.setClock(400000);
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

  initImu();

  rfid.PCD_Init();
  delay(300);

  calibrateWoodBaseline();

  Serial.println("Ready. Place robot facing the next open-field node and type g.");
}

void loop() {
  mc.resetCommandTimeout();
  handleSerialCommands();
  updateTask4OpenNavigation();
  printDebugStatus();
  delay(5);
}
