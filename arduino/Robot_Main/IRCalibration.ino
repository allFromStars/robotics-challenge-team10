// ============================================================
// IRCalibration.ino - IR Reflectance Calibration
// ============================================================
// 
// The IR array does not use one fixed raw threshold for every surface. During
// calibration, each sensor records its own minimum and maximum discharge time
// while the robot is swept over both the white floor and the black line. The
// navigation code then maps raw IR readings into a common 0-1000 scale.

unsigned long calibrationStartMs = 0;
unsigned long calibrationLastPrintMs = 0;
const unsigned long CALIBRATION_TIME_MS = 10000;
const unsigned long CALIBRATION_DIRECTION_INTERVAL_MS = 2500;
const int CALIBRATION_MOVE_SPEED = 180;
bool calibrationComplete = false;

uint16_t calibratedValues[IR_COUNT];

// Per-sensor calibration bounds. These are updated during the 12-second sweep
// and later used by Navigation.ino when calculating line position.
uint16_t sensorMin[IR_COUNT] = {0};
uint16_t sensorMax[IR_COUNT] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

void startIRCalibration() {
  // Serial.println("--- Starting IR Calibration ---");
  // Serial.println("Robot will move forward/back automatically over the line.");

  // Reset bounds before each calibration run so old lighting/surface values do
  // not affect the next trial.
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    sensorMin[i] = IR_TIMEOUT_MICROS;
    sensorMax[i] = 0;
  }

  calibrationStartMs = millis();
  calibrationLastPrintMs = 0;
  calibrationComplete = false;
}

void updateIRCalibration() {
  if (calibrationComplete) {
    stopMotors();
    return;
  }

  // Non-blocking calibration update: the main loop continues running, and this
  // function gradually records the lowest/highest raw value seen by each sensor.
  unsigned long elapsedMs = millis() - calibrationStartMs;
  bool movingForward = ((elapsedMs / CALIBRATION_DIRECTION_INTERVAL_MS) % 2) == 0;

  if (movingForward) {
    driveMotors(FORWARD_SIGN * CALIBRATION_MOVE_SPEED, FORWARD_SIGN * CALIBRATION_MOVE_SPEED);
  } else {
    driveMotors(-FORWARD_SIGN * CALIBRATION_MOVE_SPEED, -FORWARD_SIGN * CALIBRATION_MOVE_SPEED);
  }

  for (uint8_t i = 0; i < IR_COUNT; i++) {
    if (sensors.irLineArray[i] < sensorMin[i]) {
      sensorMin[i] = sensors.irLineArray[i];
    }
    if (sensors.irLineArray[i] > sensorMax[i]) {
      sensorMax[i] = sensors.irLineArray[i];
    }
  }

  // Print progress once per second so the operator knows calibration is active.
  if (millis() - calibrationLastPrintMs >= 1000) {
    calibrationLastPrintMs = millis();
    unsigned long remainingMs = (elapsedMs < CALIBRATION_TIME_MS) ? (CALIBRATION_TIME_MS - elapsedMs) : 0;
    int secondsLeft = remainingMs / 1000;
//     Serial.print("Calibrating... ");
//     Serial.print(secondsLeft);
    // Serial.println("s remaining");
  }

  // After the sweep window, the collected min/max values become the calibration
  // data used by line following.
  if (elapsedMs >= CALIBRATION_TIME_MS) {
    stopMotors();
    calibrationComplete = true;
    // Serial.println("--- IR Calibration Complete! ---");
  }
}

bool isCalibrationComplete() {
  return calibrationComplete;
}
