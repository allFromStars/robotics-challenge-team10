uint32_t tofHistory1[TOF_FILTER_SIZE] = {0}, tofHistory2[TOF_FILTER_SIZE] = {0};
uint32_t tofHistory3[TOF_FILTER_SIZE] = {0}, tofHistory4[TOF_FILTER_SIZE] = {0};
int tofIdx1 = 0, tofIdx2 = 0, tofIdx3 = 0, tofIdx4 = 0;
uint8_t rawFrame1[16], rawFrame2[16], rawFrame3[16], rawFrame4[16];

// Helper functions declared
uint32_t runTofMovingAverage(uint32_t sample, uint32_t *history, int &idx);
uint32_t parseTofFrameBytes(uint8_t *frame);
bool processTofStream(HardwareSerial &port, uint8_t *frame);
void readAllTOF();
void readIR();
void CheckRFID();
void readIMU();
bool calibrateGyroBiasZ(unsigned long calibrationMs);


bool checkI2CDevice(TwoWire &wireBus, uint8_t address) {
  wireBus.beginTransmission(address);
  return (wireBus.endTransmission() == 0); 
}

bool initIMU() {
  if (!checkI2CDevice(Wire, 0x68)) {
    return false;
  }

  myICM.begin(Wire, 0); 
  if (myICM.status != ICM_20948_Stat_Ok) {
    return false;
  }

  if (!calibrateGyroBiasZ(2000)) {
    return false;
  }
  
  sensors.yaw = 0.0;
  lastTimeMicros = micros();
  return true;
}

bool calibrateGyroBiasZ(unsigned long calibrationMs) {
  Serial.println("Calibrating IMU gyro bias...");
  gyroBiasZ = 0.0;

  int samples = 0;
  unsigned long timeoutAnchor = millis();
  while (millis() - timeoutAnchor < calibrationMs) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      gyroBiasZ += myICM.gyrZ();
      samples++;
    }
    delay(1);
  }
  
  if (samples == 0) {
    gyroBiasZ = 0.0;
    Serial.println("IMU gyro bias calibration failed: no samples.");
    return false;
  }

  gyroBiasZ /= (float)samples;
  
  Serial.print("IMU gyro bias Z: ");
  Serial.print(gyroBiasZ, 4);
  Serial.print(" dps from ");
  Serial.print(samples);
  Serial.println(" samples");

  lastTimeMicros = micros();
  return true;
}

void refreshAllSensors() {
  readAllTOF();
  readIR(); // IR stays active (uses native microcontroller pins)
  
  if (rfidOnline)  CheckRFID();
  if (imuOnline) {
    readIMU(); 
    sensors.compassNorth = getHeadingDegrees();
  }   
  
}

void readIMU() {
  if (myICM.dataReady()) {
    myICM.getAGMT();

    unsigned long currentTimeMicros = micros();
    float dt = (currentTimeMicros - lastTimeMicros) / 1000000.0; 
    lastTimeMicros = currentTimeMicros;

    if (dt <= 0.0) dt = 0.001; 

    float gyroZ = myICM.gyrZ() - gyroBiasZ;
    if (abs(gyroZ) > GYRO_DEADBAND_DPS) { 
      sensors.yaw += gyroZ * dt;
    }
  }
}

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
    while (index < 16 && millis() - start < 10) {
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
  if (tofFrontOnline && processTofStream(Serial1, rawFrame1)) {
    uint32_t raw = parseTofFrameBytes(rawFrame1);
    sensors.tof_front = (int32_t)runTofMovingAverage(raw, tofHistory1, tofIdx1) - TOF_FRONT_OFFSET;
  }
  if (tofLeftOnline && processTofStream(Serial2, rawFrame2)) {
    uint32_t raw = parseTofFrameBytes(rawFrame2);
    sensors.tof_left = (int32_t)runTofMovingAverage(raw, tofHistory2, tofIdx2) - TOF_LEFT_OFFSET;
  }
  if (tofRight1Online && processTofStream(Serial3, rawFrame3)) {
    uint32_t raw = parseTofFrameBytes(rawFrame3);
    sensors.tof_right1 = (int32_t)runTofMovingAverage(raw, tofHistory3, tofIdx3) - TOF_RIGHT1_OFFSET;
  }
  if (tofRight2Online && processTofStream(Serial4, rawFrame4)) {
    uint32_t raw = parseTofFrameBytes(rawFrame4);
    sensors.tof_right2 = (int32_t)runTofMovingAverage(raw, tofHistory4, tofIdx4) - TOF_RIGHT2_OFFSET;
  }
}

void readIR() {
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(irPins[i], OUTPUT);
    digitalWrite(irPins[i], HIGH);
  }
  delayMicroseconds(10); 

  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(irPins[i], INPUT);
    sensors.irLineArray[i] = IR_TIMEOUT_MICROS;
  }
  
  unsigned long measureStart = micros();
  bool activePinsRemaining = true;
  
  while (activePinsRemaining && (micros() - measureStart < IR_TIMEOUT_MICROS)) {
    activePinsRemaining = false;
    for (uint8_t i = 0; i < IR_COUNT; i++) {
      if (sensors.irLineArray[i] == IR_TIMEOUT_MICROS) {
        if (digitalRead(irPins[i]) == LOW) {
          sensors.irLineArray[i] = micros() - measureStart;
        } else {
          activePinsRemaining = true;
        }
      }
    }
  }
}

// Non-blocking timer variable for the printing cadence


void CheckRFID() {
  // Send Wake-Up command 
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  
  // Check if card is in the field and responds (STATUS_OK)
  if (rfid.PICC_WakeupA(bufferATQA, &bufferSize) == rfid.STATUS_OK) {
    
    // Card is present - Read its serial number.
    if (rfid.PICC_ReadCardSerial()) {
      uint32_t packedID = 0;
      for (byte i = 0; i < rfid.uid.size && i < 4; i++) {
        packedID = (packedID << 8) | rfid.uid.uidByte[i];
      }
      
      // Save
      sensors.rfidInfo = packedID; 
      rfidCardPresent = true; 

      //Put the card to sleep
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
  } 
  else {
    sensors.rfidInfo = 0; 
    rfidCardPresent = false; 
    rfid.PCD_StopCrypto1();
  }
}

float getHeadingDegrees() {
  float magX = myICM.magX();
  float magY = myICM.magY();


  float headingRadians = atan2(magY, magX);


  float headingDegrees = headingRadians * (180.0 / M_PI);

  float declinationDegrees = MAG_DECLINATION_DEG; 
  headingDegrees += declinationDegrees;


  if (headingDegrees < 0) {
    headingDegrees += 360.0;
  }
  if (headingDegrees >= 360.0) {
    headingDegrees -= 360.0;
  }

  return headingDegrees;
}

void snapYawToGrid() { //for correcting gyro drift

  // Example: 86.4 / 90 = 0.96 -> rounds to 1.0 -> 1.0 * 90 = 90.0
  // Example: 4.2 / 90 = 0.04 -> rounds to 0.0 -> 0.0 * 90 = 0.0
  
  float snappedYaw = round(sensors.yaw / 90.0) * 90.0;
  

  
  Serial.print("--- GYRO DRIFT CORRECTED --- ");
  Serial.print("Old Yaw: "); Serial.print(sensors.yaw);
  Serial.print(" | Snapped Yaw: "); Serial.println(snappedYaw);
  
  sensors.yaw = snappedYaw;
}

void DebugSensors() {
  refreshAllSensors();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= DEBUG_PRINT_INTERVAL_MS) {
    lastPrint = millis();
    
    if (debugIRMode) {
      // ==========================================
      // IR ARRAY MODE (Prints all pins)
      // ==========================================
      Serial.print("IR ARRAY [0-8]: ");
      for (int i = 0; i < IR_COUNT; i++) {
        // Formats it neatly: val, val, val
        Serial.print(sensors.irLineArray[i]);
        if (i < IR_COUNT - 1) Serial.print(", "); 
      }
      Serial.println(); // Drop to next line
    } 
    else {

      Serial.print("YAW: "); Serial.print(sensors.yaw, 1);
      Serial.print(" | COMPASS NORTH: "); Serial.print(sensors.compassNorth, 1);
      Serial.print(" | TOF: F="); Serial.print(sensors.tof_front);
      Serial.print(", L="); Serial.print(sensors.tof_left);
      Serial.print(", R1="); Serial.print(sensors.tof_right1);
      Serial.print(", R2="); Serial.print(sensors.tof_right2);
      Serial.print(" | IR Center (S5): "); Serial.print(sensors.irLineArray[4]);
      Serial.print(" | "); 

      static uint32_t lastPrintedID = 0;

      if (sensors.rfidInfo != 0) {
        if (sensors.rfidInfo != lastPrintedID) {
          Serial.print("Card detected: 0x");
          Serial.println(sensors.rfidInfo, HEX);
          lastPrintedID = sensors.rfidInfo;
        } else {
          Serial.println("Card detected: same card");
        }
      } 
      else {
        Serial.println("no card detected");
        lastPrintedID = 0; 
      }
    }
  }
}
