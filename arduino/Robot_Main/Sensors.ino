// Set TOF sensor history array size to the set sample size
uint32_t tofHistory1[TOF_FILTER_SIZE] = {0}, tofHistory2[TOF_FILTER_SIZE] = {0};
uint32_t tofHistory3[TOF_FILTER_SIZE] = {0}, tofHistory4[TOF_FILTER_SIZE] = {0};
int tofIdx1 = 0, tofIdx2 = 0, tofIdx3 = 0, tofIdx4 = 0;
uint8_t rawFrame1[16], rawFrame2[16], rawFrame3[16], rawFrame4[16];


void refreshAllSensors() {
  readAllTOF();
  readIR();
  CheckRFID();
}


// Find average distance data from Tof sensors
uint32_t runTofMovingAverage(uint32_t sample, uint32_t *history, int &idx) {
  history[idx] = sample;
  idx = (idx + 1) % TOF_FILTER_SIZE;
  uint64_t sum = 0;
  for (int i = 0; i < TOF_FILTER_SIZE; i++) sum += history[i];
  return sum / TOF_FILTER_SIZE;
}

uint32_t parseTofFrameBytes(uint8_t *frame) {
  return ((uint32_t)frame[8]) | ((uint32_t)frame[9] << 8) | ((uint32_t)frame[10] << 16);
}

bool processTofStream(HardwareSerial &port, uint8_t *frame) {
  while (port.available()) {
    uint8_t b = port.read();
    if (b != 0x57) continue;
    frame[0] = b;
    unsigned long start = millis();
    int index = 1;
    while (index < 16 && millis() - start < 20) {
      if (port.available()) { frame[index] = port.read(); index++; }
    }
    if (index < 16 || frame[1] != 0x00 || frame[2] != 0xFF) return false;
    uint8_t checksum = 0;
    for (int i = 0; i < 15; i++) checksum += frame[i];
    if (checksum != frame[15]) return false;
    return true;
  }
  return false;
}

void readAllTOF() {
  if (processTofStream(Serial1, rawFrame1)) {
    uint32_t raw = parseTofFrameBytes(rawFrame1);
    sensors.tof_front = (int32_t)runTofMovingAverage(raw, tofHistory1, tofIdx1) - offset1;
  }
  if (processTofStream(Serial2, rawFrame2)) {
    uint32_t raw = parseTofFrameBytes(rawFrame2);
    sensors.tof_left = (int32_t)runTofMovingAverage(raw, tofHistory2, tofIdx2) - offset2;
  }
  if (processTofStream(Serial3, rawFrame3)) {
    uint32_t raw = parseTofFrameBytes(rawFrame3);
    sensors.tof_right1 = (int32_t)runTofMovingAverage(raw, tofHistory3, tofIdx3) - offset3;
  }
  if (processTofStream(Serial4, rawFrame4)) {
    uint32_t raw = parseTofFrameBytes(rawFrame4);
    sensors.tof_right2 = (int32_t)runTofMovingAverage(raw, tofHistory4, tofIdx4) - offset4;
  }
}

// Get reflectance sensor readings
void readIR() {
  static enum { CHARGE, MEASURE } irState = CHARGE;
  static unsigned long stateStartTimeMicros = 0;

  if (irState == CHARGE) {
    for (uint8_t i = 0; i < IR_COUNT; i++) {
      pinMode(irPins[i], OUTPUT);
      digitalWrite(irPins[i], HIGH);
    }
    stateStartTimeMicros = micros();
    irState = MEASURE;
  } 
  else if (irState == MEASURE) {
    if (micros() - stateStartTimeMicros >= 10) {
      for (uint8_t i = 0; i < IR_COUNT; i++) {
        pinMode(irPins[i], INPUT);
        irValues[i] = IR_TIMEOUT_MICROS;
      }
      
      unsigned long measureStart = micros();
      bool activePinsRemaining = true;
      
      
      while (activePinsRemaining && (micros() - measureStart < IR_TIMEOUT_MICROS)) {
        activePinsRemaining = false;
        for (uint8_t i = 0; i < IR_COUNT; i++) {
          if (irValues[i] == IR_TIMEOUT_MICROS) {
            if (digitalRead(irPins[i]) == LOW) {
              irValues[i] = micros() - measureStart;
            } else {
              activePinsRemaining = true;
            }
          }
        }
      }
      irState = CHARGE; // Reset cycle for next pass
    }
  }
}

// Check for RFID 
void CheckRFID() {
  // if no card, return back to main loop
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    rfidCardPresent = false;
    return;
  }
  
  rfidCardPresent = true;
  
  // Clean shutdown steps to clear reader flags for subsequent detections
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}