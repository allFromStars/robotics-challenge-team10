#include <Wire.h>
#include "ICM_20948.h"
#include <Motoron.h>
#include <MFRC522_I2C.h>

// ============================================================
// Task 2: Base Exit Route + IMU 90 Turns + RFID Stop
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
// -> stop on first RFID tag detected
//
// This version uses the ICM-20948 gyroscope yaw integration for turning.
// It keeps the early line-stop logic during turns.

// --------------------
// Motoron setup
// --------------------

MotoronI2C mc(16);

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

const int BASE_SPEED = 280;
const int SLOW_SPEED = 180;
const int START_SPEED = 140;
const int ALIGN_SPEED = 140;
const int SEARCH_SPEED = 240;
const int MAX_SPEED = 360;
const int EXIT_ROUTE_SPEED = 240;

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

const int TURN_MOTOR_MAX_LIMIT = 300;
const int TURN_MOTOR_MIN_LIMIT = 200;
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

// Only needed when STOP_ON_ANY_RFID_AFTER_SECOND_TURN is false.
const String TARGET_B_UID = "REPLACE_WITH_B_UID";

const bool STOP_ON_ANY_RFID_AFTER_SECOND_TURN = true;

String latestUid = "";
bool anyRfidDetected = false;
bool bTagDetected = false;

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
  T2_ALIGN_ON_B_TAG,
  T2_DONE,
  T2_FAILSAFE_STOP
};

Task2State task2State = T2_IDLE;

unsigned long stateStartMs = 0;
unsigned long taskStartMs = 0;
unsigned long bTagDetectedMs = 0;
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

const unsigned long RFID_STOP_OFFSET_MS = 150;
const unsigned long TASK2_TIMEOUT_MS = 45000;
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
const unsigned long LEFT_TURN_IGNORE_LINE_MS = 250;

// Early line-stop is only allowed after the robot has already turned a
// reasonable amount. Right turn starts from a junction, so it uses IMU only.
const float LEFT_TURN_MIN_EARLY_STOP_DEG = 45.0;

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
  return lineDetected &&
         !baseJunctionDetected &&
         abs(lastError) <= REACQUIRE_MAX_ABS_ERROR &&
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
  bTagDetected = STOP_ON_ANY_RFID_AFTER_SECOND_TURN || latestUid == TARGET_B_UID;

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// --------------------
// Movement behaviours
// --------------------

void followLinePD(int baseSpeed) {
  if (!lineDetected) {
    if (lastError < 0) {
      turnInPlace(-1, SEARCH_SPEED);
    } else {
      turnInPlace(1, SEARCH_SPEED);
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
  bTagDetectedMs = 0;
  lineLostStartMs = 0;
  latestUid = "";
  anyRfidDetected = false;
  bTagDetected = false;
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
    enterFailsafe("Task 2 exceeded 45 seconds");
    return;
  }

  switch (task2State) {
    case T2_IDLE:
      stopMotors();
      break;

    case T2_LEAVE_DEPLOYMENT_AREA:
      driveForwardMagnitudes(START_SPEED, START_SPEED);

      if (lineDetected && millis() - stateStartMs > LEAVE_DEPLOYMENT_MIN_MS) {
        enterState(T2_FOLLOW_DEPLOYMENT_LINE);
      }
      break;

    case T2_FOLLOW_DEPLOYMENT_LINE:
      followLinePD(BASE_SPEED);

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
      followLinePD(SLOW_SPEED);

      if (lineCenteredAfterReacquire()) {
        enterState(T2_FOLLOW_EXIT_ROUTE_STRAIGHT);
      }

      if (millis() - stateStartMs > REACQUIRE_TIMEOUT_MS) {
        enterFailsafe("Could not reacquire exit route after right turn");
      }
      break;

    case T2_FOLLOW_EXIT_ROUTE_STRAIGHT:
      followLinePD(EXIT_ROUTE_SPEED);

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
      if (imuTurnPassedAngle(LEFT_TURN_MIN_EARLY_STOP_DEG) &&
          validLineForEarlyTurnStop(LEFT_TURN_IGNORE_LINE_MS)) {
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
      driveForwardMagnitudes(SLOW_SPEED, SLOW_SPEED);

      if (lineReacquired()) {
        lastError = 0;
        enterState(T2_APPROACH_B_TAG);
      }

      if (millis() - stateStartMs > REACQUIRE_TIMEOUT_MS) {
        enterFailsafe("Could not reacquire B line after left turn");
      }
      break;

    case T2_APPROACH_B_TAG:
      followLinePD(SLOW_SPEED);

      if (bTagDetected) {
        bTagDetectedMs = millis();
        enterState(T2_ALIGN_ON_B_TAG);
      }

      if (lineLostTooLong()) {
        enterFailsafe("Line lost while approaching B tag");
      }
      break;

    case T2_ALIGN_ON_B_TAG:
      driveForwardMagnitudes(ALIGN_SPEED, ALIGN_SPEED);

      if (millis() - bTagDetectedMs > RFID_STOP_OFFSET_MS) {
        stopMotors();
        enterState(T2_DONE);
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
  Serial.println("Task 2: scripted base exit route + IMU 90 turns + RFID stop");
  Serial.println("Mechanical control button: D8 to GND.");
  Serial.println("Press button once to start Task 2.");
  Serial.println("Press button again to stop motors.");
  Serial.println("Current mode: stop on any RFID after second turn.");

  pinMode(CONTROL_BUTTON_PIN, INPUT_PULLUP);

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
  handleControlButton();
  handleSerialCommands();
  runTask2();
  delay(5);
}
