// ============================================================
// IRCalibration.ino - IR Reflectance Calibration
// ============================================================
// 
// The IR array does not use one fixed raw threshold for every surface. During
// calibration, each sensor records its own minimum and maximum discharge time
// while the robot is swept over both the white floor and the black line. The
// navigation code then maps raw IR readings into a common 0-1000 scale.

unsigned long calibrationStartMs = 0;
const unsigned long CALIBRATION_TIME_MS = 12000;
bool calibrationComplete = false;

uint16_t calibratedValues[IR_COUNT];

// Per-sensor calibration bounds. These are updated during the 12-second sweep
// and later used by Navigation.ino when calculating line position.
uint16_t sensorMin[IR_COUNT] = {0};
uint16_t sensorMax[IR_COUNT] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

void startIRCalibration() {
  Serial.println("--- Starting IR Calibration ---");
  Serial.println("Sweep the sensor array over the white floor and black line!");

  // Reset bounds before each calibration run so old lighting/surface values do
  // not affect the next trial.
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    sensorMin[i] = IR_TIMEOUT_MICROS;
    sensorMax[i] = 0;
  }

  calibrationStartMs = millis();
  calibrationComplete = false;
}

void updateIRCalibration() {
  // Non-blocking calibration update: the main loop continues running, and this
  // function gradually records the lowest/highest raw value seen by each sensor.
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    if (sensors.irLineArray[i] < sensorMin[i]) {
      sensorMin[i] = sensors.irLineArray[i];
    }
    if (sensors.irLineArray[i] > sensorMax[i]) {
      sensorMax[i] = sensors.irLineArray[i];
    }
  }

  // Print progress once per second so the operator knows calibration is active.
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    int secondsLeft = (CALIBRATION_TIME_MS - (millis() - calibrationStartMs)) / 1000;
    Serial.print("Calibrating... ");
    Serial.print(secondsLeft);
    Serial.println("s remaining");
  }

  // After the sweep window, the collected min/max values become the calibration
  // data used by line following.
  if (millis() - calibrationStartMs >= CALIBRATION_TIME_MS) {
    calibrationComplete = true;
    Serial.println("--- IR Calibration Complete! ---");
  }
}

bool isCalibrationComplete() {
  return calibrationComplete;
}
