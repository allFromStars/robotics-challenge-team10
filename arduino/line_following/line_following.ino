#include <Wire.h>
#include <Motoron.h>
#include <QTRSensors.h>

// ---------------- MOTORON SETUP ----------------
MotoronI2C mc(16);

const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;

const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;

// Adjust these after testing
const int BASE_SPEED = 300;
const int SLOW_SPEED = 120;
const int TURN_SPEED = 260;

// ---------------- QTR SETUP ----------------
QTRSensors qtr;

const uint8_t SensorCount = 9;
uint16_t sensorValues[SensorCount];

// Change these pins if your wiring is different
const uint8_t qtrPins[SensorCount] = {
  22, 23, 24, 25, 26, 27, 28, 29, 30
};

// Threshold after calibration.
// qtr.readCalibrated() gives values around 0–1000.
// For black line, values are usually high when using readLineBlack logic.
const int BLACK_THRESHOLD = 500;

// ---------------- LINE STATE ----------------
enum LastDirection {
  LAST_LEFT,
  LAST_RIGHT,
  LAST_STRAIGHT
};

LastDirection lastDirection = LAST_STRAIGHT;

// ---------------- MOTOR FUNCTIONS ----------------
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  mc.setSpeed(LEFT_MOTOR_CHANNEL, leftSpeed * LEFT_DIR);
  mc.setSpeed(RIGHT_MOTOR_CHANNEL, rightSpeed * RIGHT_DIR);
}

void moveForward() {
  setMotorSpeeds(BASE_SPEED, BASE_SPEED);
}

void turnLeft() {
  setMotorSpeeds(SLOW_SPEED, TURN_SPEED);
}

void turnRight() {
  setMotorSpeeds(TURN_SPEED, SLOW_SPEED);
}

void searchLeft() {
  setMotorSpeeds(-SLOW_SPEED, SLOW_SPEED);
}

void searchRight() {
  setMotorSpeeds(SLOW_SPEED, -SLOW_SPEED);
}

void stopMotors() {
  setMotorSpeeds(0, 0);
}

// ---------------- SENSOR LOGIC ----------------
bool isBlack(int index) {
  return sensorValues[index] > BLACK_THRESHOLD;
}

bool leftOnLine() {
  return isBlack(0) || isBlack(1) || isBlack(2);
}

bool centreOnLine() {
  return isBlack(3) || isBlack(4) || isBlack(5);
}

bool rightOnLine() {
  return isBlack(6) || isBlack(7) || isBlack(8);
}

bool anySensorBlack() {
  for (int i = 0; i < SensorCount; i++) {
    if (isBlack(i)) {
      return true;
    }
  }
  return false;
}

bool allSensorsBlack() {
  for (int i = 0; i < SensorCount; i++) {
    if (!isBlack(i)) {
      return false;
    }
  }
  return true;
}

void printSensorValues() {
  for (int i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();
}

// ---------------- CALIBRATION ----------------
void calibrateQTR() {
  Serial.println("Calibrating QTR sensors...");
  Serial.println("Move the sensor array across black line and white floor.");

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(5);
  }

  Serial.println("Calibration complete.");
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);
  delay(1000);

  Wire.begin();

  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();

  mc.setMaxAcceleration(LEFT_MOTOR_CHANNEL, 200);
  mc.setMaxDeceleration(LEFT_MOTOR_CHANNEL, 300);
  mc.setMaxAcceleration(RIGHT_MOTOR_CHANNEL, 200);
  mc.setMaxDeceleration(RIGHT_MOTOR_CHANNEL, 300);

  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);

  calibrateQTR();

  stopMotors();

  Serial.println("Bang-bang line follower ready.");
  delay(1000);
}

// ---------------- MAIN LOOP ----------------
void loop() {
  qtr.readCalibrated(sensorValues);

  printSensorValues();

  if (allSensorsBlack()) {
    // Possible intersection.
    // For now, just keep going forward.
    moveForward();
    lastDirection = LAST_STRAIGHT;
  }
  else if (centreOnLine()) {
    moveForward();
    lastDirection = LAST_STRAIGHT;
  }
  else if (leftOnLine()) {
    turnLeft();
    lastDirection = LAST_LEFT;
  }
  else if (rightOnLine()) {
    turnRight();
    lastDirection = LAST_RIGHT;
  }
  else {
    // Line lost: recover using last known direction.
    if (lastDirection == LAST_LEFT) {
      searchLeft();
    }
    else if (lastDirection == LAST_RIGHT) {
      searchRight();
    }
    else {
      moveForward();
    }
  }

  delay(10);
}