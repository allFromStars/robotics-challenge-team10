const uint8_t SensorCount = 9;

const uint8_t sensorPins[SensorCount] = {
  22, 23, 24, 25, 26, 27, 28, 29, 30
};

uint16_t sensorValues[SensorCount];

const uint16_t timeout = 2500; // microseconds

void readQTR_RC() {
  //charge all sensor capacitors
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }

  delayMicroseconds(10);

  // switch pins to input and measure discharge time
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

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(10);
  }

  delay(1000);

  Serial.println("QTR-HD-09RC manual RC reading test");
  Serial.println("Higher value = darker / weaker reflection");
  Serial.println("Lower value  = lighter / stronger reflection");
  Serial.println();
}

void loop() {
  readQTR_RC();

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensorValues[i]);

    if (i < SensorCount - 1) {
      Serial.print(" | ");
    }
  }

  Serial.println();
  delay(300);
}
