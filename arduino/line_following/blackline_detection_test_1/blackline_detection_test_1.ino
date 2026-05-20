const uint8_t SensorCount = 9;

const uint8_t sensorPins[SensorCount] = {
  22, 23, 24, 25, 26, 27, 28, 29, 30
};

uint16_t sensorValues[SensorCount];      // raw readings
uint16_t calibratedValues[SensorCount];  // calibrated readings from 0 to 1000

uint16_t sensorMin[SensorCount];
uint16_t sensorMax[SensorCount];

const uint16_t timeout = 5000; // microseconds

// Change this if you want longer or shorter calibration
const unsigned long CALIBRATION_TIME_MS = 12000; // 12 seconds

// Tune this if line detection is too sensitive or not sensitive enough
const uint16_t LINE_THRESHOLD = 200;

// For 9 sensors, position goes from 0 to 8000
// 4000 means the line is roughly in the centre
uint16_t lastLinePosition = 4000;

void readQTR_RC() {
  // charge all sensor capacitors
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }

  delayMicroseconds(10);

  // switch pins to input and measure how long they take to discharge
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
  // reset min and max values before calibration starts
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorMin[i] = timeout;
    sensorMax[i] = 0;
  }

  Serial.println("Calibration starting...");
  Serial.println("Move the sensor over BOTH the wood surface and the black line.");
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
  Serial.println();

  Serial.println("Minimum raw values seen during calibration:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorMin[i]);
    Serial.print('\t');
  }
  Serial.println();

  Serial.println("Maximum raw values seen during calibration:");
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

      // convert raw reading into a 0 to 1000 scale
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

    // sensor positions are 0, 1000, 2000, ... 8000
    weightedSum += (uint32_t)value * (i * 1000);
    total += value;
  }

  // if line is lost, keep the last known position
  if (total < 50) {
    return lastLinePosition;
  }

  lastLinePosition = weightedSum / total;
  return lastLinePosition;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("QTR-HD-09RC manual calibrated line reading test");
  Serial.println("Pins used: D22 to D30");
  Serial.println("CTRL pins connected to 3.3V, so emitters are always ON.");
  Serial.println("No QTR library is used in this code.");
  Serial.println();

  calibrateSensors();

  Serial.println("Now printing calibrated sensor values and line position.");
  Serial.println("0 = wood / lighter surface");
  Serial.println("1000 = black line / darker surface");
  Serial.println("Position range: 0 to 8000, centre is around 4000");
  Serial.println();
}

void loop() {
  readQTR_RC();
  calculateCalibratedValues();

  bool lineDetected = isLineDetected();
  uint16_t position = readLineBlackManual();

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(calibratedValues[i]);

    if (i < SensorCount - 1) {
      Serial.print(" | ");
    }
  }

  if (lineDetected) {
    Serial.print(" | Position: ");
    Serial.println(position);
  } else {
    Serial.println(" | NO LINE DETECTED");
  }

  delay(250);
}