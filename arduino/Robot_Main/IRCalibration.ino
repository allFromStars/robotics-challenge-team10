// ============================================================
// IRCalibration.ino - IR Reflectance Calibration
// ============================================================

unsigned long calibrationStartMs = 0;
const unsigned long CALIBRATION_TIME_MS = 12000;
bool calibrationComplete = false;

uint16_t calibratedValues[IR_COUNT];
uint16_t sensorMin[IR_COUNT] = {0};
uint16_t sensorMax[IR_COUNT] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

void startIRCalibration() {
  Serial.println("--- Starting IR Calibration ---");
  Serial.println("Sweep the sensor array over the white floor and black line!");

  for (uint8_t i = 0; i < IR_COUNT; i++) {
    sensorMin[i] = IR_TIMEOUT_MICROS;
    sensorMax[i] = 0;
  }

  calibrationStartMs = millis();
  calibrationComplete = false;
}

void updateIRCalibration() {
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    if (sensors.irLineArray[i] < sensorMin[i]) {
      sensorMin[i] = sensors.irLineArray[i];
    }
    if (sensors.irLineArray[i] > sensorMax[i]) {
      sensorMax[i] = sensors.irLineArray[i];
    }
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    int secondsLeft = (CALIBRATION_TIME_MS - (millis() - calibrationStartMs)) / 1000;
    Serial.print("Calibrating... ");
    Serial.print(secondsLeft);
    Serial.println("s remaining");
  }

  if (millis() - calibrationStartMs >= CALIBRATION_TIME_MS) {
    calibrationComplete = true;
    Serial.println("--- IR Calibration Complete! ---");
  }
}

bool isCalibrationComplete() {
  return calibrationComplete;
}
