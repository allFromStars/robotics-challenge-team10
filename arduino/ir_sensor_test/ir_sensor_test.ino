const uint8_t SensorCount = 9;

const uint8_t sensorPins[SensorCount] = {
  2, 3, 4, 5, 6, 7, 8, 9, 10
};

uint16_t sensorValues[SensorCount];

const uint16_t timeout = 2500; // microseconds


void readQTR_RC() {

  // Step 1: charge all sensor capacitors
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }

  delayMicroseconds(10);

  // Step 2: switch pins to input and time how long they stay HIGH
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
  Serial.println("S1\tS2\tS3\tS4\tS5\tS6\tS7\tS8\tS9");
}


void loop() {

  readQTR_RC();

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  Serial.println();

  delay(300);
}