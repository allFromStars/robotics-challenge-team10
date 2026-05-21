#include <Wire.h>
#include <Motoron.h>

// =====================
// MOTORON SETUP
// =====================

MotoronI2C mc(16);

const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;

const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;

// On our current robot code, forward movement uses negative logical speeds.
const int FORWARD_SIGN = -1;

// Start conservatively. Increase only after the robot follows a straight line.
const int BASE_SPEED = 110;
const int MAX_SPEED = 220;
const int SEARCH_SPEED = 90;

// Tune these on the floor at low speed.
float Kp = 0.035;
float Kd = 0.10;

// =====================
// IR / QTR RC SETUP
// =====================

const uint8_t SensorCount = 9;

const uint8_t sensorPins[SensorCount] = {
  22, 23, 24, 25, 26, 27, 28, 29, 30
};

uint16_t sensorValues[SensorCount];      // raw RC discharge readings
uint16_t calibratedValues[SensorCount];  // calibrated readings from 0 to 1000

uint16_t sensorMin[SensorCount];
uint16_t sensorMax[SensorCount];

const uint16_t timeout = 2500; // microseconds

const unsigned long CALIBRATION_TIME_MS = 10000;

// For 9 sensors, position goes from 0 to 8000.
const int LINE_CENTER = 4000;

// If any calibrated sensor is above this, we assume some line is visible.
const uint16_t LINE_THRESHOLD = 200;

// If the weighted sum is below this, the line signal is too weak.
const uint16_t LINE_TOTAL_THRESHOLD = 50;

uint16_t lastLinePosition = LINE_CENTER;
int lastError = 0;

bool robotEnabled = false;
unsigned long lastDebugPrintTime = 0;

// =====================
// MOTOR HELPERS
// =====================

int clampMagnitude(int speed) {
  if (speed > MAX_SPEED) {
    return MAX_SPEED;
  }

  if (speed < 0) {
    return 0;
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

void searchLeft() {
  // Matches the existing robot_demo convention for left turn:
  // driveMotors(-FORWARD_SPEED, FORWARD_SPEED), where FORWARD_SPEED is negative.
  driveMotors(SEARCH_SPEED, -SEARCH_SPEED);
}

void searchRight() {
  driveMotors(-SEARCH_SPEED, SEARCH_SPEED);
}

// =====================
// SENSOR READING
// =====================

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
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorMin[i] = timeout;
    sensorMax[i] = 0;
  }

  Serial.println("Calibration starting...");
  Serial.println("Move the sensor array across BOTH the floor and the black line.");
  Serial.println("Make sure all 9 sensors see both surfaces.");
  Serial.println();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  unsigned long startTime = millis();
  int lastSecondShown = -1;

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

    unsigned long elapsedTime = millis() - startTime;
    unsigned long remainingTime = CALIBRATION_TIME_MS - elapsedTime;
    int secondsRemaining = (remainingTime + 999) / 1000;

    if (secondsRemaining != lastSecondShown) {
      Serial.print("Calibrating... ");
      Serial.print(secondsRemaining);
      Serial.println(" seconds remaining");
      lastSecondShown = secondsRemaining;
    }

    delay(10);
  }

  digitalWrite(LED_BUILTIN, LOW);

  Serial.println();
  Serial.println("Calibration complete.");

  Serial.println("Minimum raw values:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorMin[i]);
    Serial.print('\t');
  }
  Serial.println();

  Serial.println("Maximum raw values:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorMax[i]);
    Serial.print('\t');
  }
  Serial.println();
  Serial.println();
}

void calculateCalibratedValues() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorMax[i] <= sensorMin[i]) {
      calibratedValues[i] = 0;
    } else {
      long value = sensorValues[i];
      value = ((value - sensorMin[i]) * 1000L) / (sensorMax[i] - sensorMin[i]);

      if (value < 0) {
        value = 0;
      }

      if (value > 1000) {
        value = 1000;
      }

      calibratedValues[i] = value;
    }
  }
}

bool isLineDetected() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (calibratedValues[i] > LINE_THRESHOLD) {
      return true;
    }
  }

  return false;
}

uint16_t readLineBlackManual() {
  uint32_t weightedSum = 0;
  uint32_t total = 0;

  for (uint8_t i = 0; i < SensorCount; i++) {
    uint16_t value = calibratedValues[i];
    weightedSum += (uint32_t)value * (i * 1000);
    total += value;
  }

  if (total < LINE_TOTAL_THRESHOLD) {
    return lastLinePosition;
  }

  lastLinePosition = weightedSum / total;
  return lastLinePosition;
}

int countActiveSensors() {
  int activeCount = 0;

  for (uint8_t i = 0; i < SensorCount; i++) {
    if (calibratedValues[i] > LINE_THRESHOLD) {
      activeCount++;
    }
  }

  return activeCount;
}

// =====================
// LINE FOLLOWING
// =====================

void followLinePD() {
  readQTR_RC();
  calculateCalibratedValues();

  bool lineDetected = isLineDetected();
  uint16_t position = readLineBlackManual();
  int activeCount = countActiveSensors();

  if (!lineDetected) {
    if (lastError < 0) {
      searchLeft();
    } else {
      searchRight();
    }

    printDebug(position, lastError, 0, 0, activeCount, false);
    return;
  }

  int error = (int)position - LINE_CENTER;
  int derivative = error - lastError;
  int correction = (int)(Kp * error + Kd * derivative);

  int leftMagnitude = BASE_SPEED + correction;
  int rightMagnitude = BASE_SPEED - correction;

  driveForwardMagnitudes(leftMagnitude, rightMagnitude);

  printDebug(position, error, leftMagnitude, rightMagnitude, activeCount, true);

  lastError = error;
}

void printDebug(
  uint16_t position,
  int error,
  int leftMagnitude,
  int rightMagnitude,
  int activeCount,
  bool lineDetected
) {
  if (millis() - lastDebugPrintTime < 200) {
    return;
  }

  lastDebugPrintTime = millis();

  Serial.print("line=");
  Serial.print(lineDetected ? "yes" : "no");

  Serial.print(" active=");
  Serial.print(activeCount);

  Serial.print(" pos=");
  Serial.print(position);

  Serial.print(" error=");
  Serial.print(error);

  Serial.print(" L=");
  Serial.print(leftMagnitude);

  Serial.print(" R=");
  Serial.println(rightMagnitude);
}

void handleSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  char command = Serial.read();

  if (command == 'g' || command == 'G') {
    robotEnabled = true;
    lastError = 0;
    lastLinePosition = LINE_CENTER;
    Serial.println("GO: PD line following enabled.");
  }

  if (command == 's' || command == 'S') {
    robotEnabled = false;
    stopMotors();
    Serial.println("STOP: motors disabled.");
  }
}

// =====================
// SETUP / LOOP
// =====================

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(10);
  }

  delay(1000);

  Serial.println();
  Serial.println("Manual QTR blackline PD control test");
  Serial.println("Pins used: D22 to D30");
  Serial.println("Upload, calibrate, then type 'g' to go or 's' to stop.");
  Serial.println();

  Wire1.begin();
  delay(500);

  mc.setBus(&Wire1);
  mc.reinitialize();
  mc.clearResetFlag();
  mc.setCommandTimeoutMilliseconds(1000);

  mc.setMaxAcceleration(LEFT_MOTOR_CHANNEL, 120);
  mc.setMaxDeceleration(LEFT_MOTOR_CHANNEL, 250);
  mc.setMaxAcceleration(RIGHT_MOTOR_CHANNEL, 120);
  mc.setMaxDeceleration(RIGHT_MOTOR_CHANNEL, 250);

  stopMotors();

  calibrateSensors();

  Serial.println("Ready.");
  Serial.println("Type 'g' in Serial Monitor to start low-speed PD line following.");
  Serial.println("Type 's' to stop.");
  Serial.println();
}

void loop() {
  mc.resetCommandTimeout();
  handleSerialCommands();

  if (!robotEnabled) {
    stopMotors();
    delay(20);
    return;
  }

  followLinePD();
  delay(5);
}

