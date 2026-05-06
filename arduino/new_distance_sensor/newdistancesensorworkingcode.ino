// Waveshare TOF Laser Range Sensor Mini
// Arduino UNO R4 UART Distance Only With Unit
// Wiring:
// Sensor VCC -> Arduino 5V
// Sensor GND -> Arduino GND
// Sensor TX  -> Arduino D0 / RX
// Sensor RX  -> Arduino D1 / TX

const unsigned long SENSOR_BAUD = 921600;

uint8_t frame[16];

bool readTOFFrame() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    if (b == 0x57) {
      frame[0] = b;

      unsigned long startTime = millis();
      int index = 1;

      while (index < 16 && millis() - startTime < 50) {
        if (Serial1.available()) {
          frame[index] = Serial1.read();
          index++;
        }
      }

      if (index < 16) return false;

      if (frame[1] != 0x00 || frame[2] != 0xFF) return false;

      uint8_t checksum = 0;
      for (int i = 0; i < 15; i++) {
        checksum += frame[i];
      }

      if (checksum != frame[15]) return false;

      return true;
    }
  }

  return false;
}

uint32_t getDistanceMM() {
  return ((uint32_t)frame[8]) |
         ((uint32_t)frame[9] << 8) |
         ((uint32_t)frame[10] << 16);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(SENSOR_BAUD);

  delay(2000);
}

void loop() {
  static unsigned long lastPrintTime = 0;

  if (readTOFFrame()) {
    if (millis() - lastPrintTime >= 200) {
      uint32_t distanceMM = getDistanceMM();

      Serial.print(distanceMM);
      Serial.println(" mm");

      lastPrintTime = millis();
    }
  }
}