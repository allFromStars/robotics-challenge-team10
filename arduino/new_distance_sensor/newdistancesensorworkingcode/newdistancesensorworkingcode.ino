// Waveshare TOF Laser Range Sensor Mini
// Arduino GIGA R1 - 4 UART sensors
//
// Sensor 1:
//   TX -> D0  / RX0
//   RX -> D1  / TX0
//
// Sensor 2:
//   TX -> D19 / RX1
//   RX -> D18 / TX1
//
// Sensor 3:
//   TX -> D17 / RX2
//   RX -> D16 / TX2
//
// Sensor 4:
//   TX -> D15 / RX3
//   RX -> D14 / TX3
//
// All sensors:
//   VCC -> 5V
//   GND -> GND

const unsigned long SENSOR_BAUD = 921600;

uint8_t frame1[16];
uint8_t frame2[16];
uint8_t frame3[16];
uint8_t frame4[16];

uint32_t distance1 = 0;
uint32_t distance2 = 0;
uint32_t distance3 = 0;
uint32_t distance4 = 0;

bool readTOFFrame(HardwareSerial &port, uint8_t *frame) {
  while (port.available()) {
    uint8_t b = port.read();

    // Frame starts with 0x57
    if (b != 0x57) {
      continue;
    }

    frame[0] = b;

    unsigned long startTime = millis();
    int index = 1;

    // Read remaining 15 bytes
    while (index < 16 && millis() - startTime < 50) {
      if (port.available()) {
        frame[index] = port.read();
        index++;
      }
    }

    if (index < 16) {
      return false;
    }

    // Basic frame check
    if (frame[1] != 0x00 || frame[2] != 0xFF) {
      return false;
    }

    // Checksum: sum of first 15 bytes, low 8 bits
    uint8_t checksum = 0;
    for (int i = 0; i < 15; i++) {
      checksum += frame[i];
    }

    if (checksum != frame[15]) {
      return false;
    }

    return true;
  }

  return false;
}

uint32_t getDistanceMM(uint8_t *frame) {
  // Distance is 24-bit little-endian
  return ((uint32_t)frame[8]) |
         ((uint32_t)frame[9] << 8) |
         ((uint32_t)frame[10] << 16);
}

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(10);
  }

  Serial1.begin(SENSOR_BAUD);  // D0 / D1
  Serial2.begin(SENSOR_BAUD);  // D19 / D18
  Serial3.begin(SENSOR_BAUD);  // D17 / D16
  Serial4.begin(SENSOR_BAUD);  // D15 / D14

  delay(2000);

  Serial.println("4 TOF sensors UART test");
  Serial.println("S1: D0/D1 | S2: D19/D18 | S3: D17/D16 | S4: D15/D14");
}

void loop() {
  if (readTOFFrame(Serial1, frame1)) {
    distance1 = getDistanceMM(frame1);
  }

  if (readTOFFrame(Serial2, frame2)) {
    distance2 = getDistanceMM(frame2);
  }

  if (readTOFFrame(Serial3, frame3)) {
    distance3 = getDistanceMM(frame3);
  }

  if (readTOFFrame(Serial4, frame4)) {
    distance4 = getDistanceMM(frame4);
  }

  static unsigned long lastPrintTime = 0;

  if (millis() - lastPrintTime >= 200) {
    Serial.print("Sensor 1: ");
    Serial.print(distance1);
    Serial.print(" mm");

    Serial.print(" | Sensor 2: ");
    Serial.print(distance2);
    Serial.print(" mm");

    Serial.print(" | Sensor 3: ");
    Serial.print(distance3);
    Serial.print(" mm");

    Serial.print(" | Sensor 4: ");
    Serial.print(distance4);
    Serial.println(" mm");

    lastPrintTime = millis();
  }
}