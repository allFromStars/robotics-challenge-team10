uint32_t tofHistory1[TOF_FILTER_SIZE] = {0}, tofHistory2[TOF_FILTER_SIZE] = {0};
uint32_t tofHistory3[TOF_FILTER_SIZE] = {0}, tofHistory4[TOF_FILTER_SIZE] = {0};
int tofIdx1 = 0, tofIdx2 = 0, tofIdx3 = 0, tofIdx4 = 0;
bool tofHistoryReady1 = false, tofHistoryReady2 = false;
bool tofHistoryReady3 = false, tofHistoryReady4 = false;
uint8_t rawFrame1[16], rawFrame2[16], rawFrame3[16], rawFrame4[16];


// Sensor module overview

// This file keeps all sensor polling and sensor calibration helpers in one
// place. Robot_Main.ino calls
// refreshAllSensors() every loop, so control and safety decisions use the
// latest available IR, RFID, TOF, and IMU readings.

// Helper functions declared
uint32_t runTofMovingAverage(uint32_t sample, uint32_t *history, int &idx, bool &historyReady);
uint32_t parseTofFrameBytes(uint8_t *frame);
bool processTofStream(HardwareSerial &port, uint8_t *frame);
void readAllTOF();
void readIR();
void CheckRFID();
void readIMU();
bool calibrateGyroBiasZ(unsigned long calibrationMs);


bool checkI2CDevice(TwoWire &wireBus, uint8_t address) {
  // Quick hardware presence check used during startup diagnostics.
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
  // Testing/calibration evidence: the robot samples stationary gyro Z readings
  // and stores the average bias. Later yaw integration subtracts this value so
  // small sensor offset does not immediately become heading drift.
  // Serial.println("Calibrating IMU gyro bias...");
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
    // Serial.println("IMU gyro bias calibration failed: no samples.");
    return false;
  }

  gyroBiasZ /= (float)samples;
  
//   Serial.print("IMU gyro bias Z: ");
//   Serial.print(gyroBiasZ, 4);
//   Serial.print(" dps from ");
//   Serial.print(samples);
  // Serial.println(" samples");

  lastTimeMicros = micros();
  return true;
}

void refreshAllSensors() {
  readAllTOF();
  readIR(); 

  if (rfidOnline) {
    CheckRFID(); 
    
    if (sensors.rfidInfo != 0 &&
        currentState != STATE_TASK2_BASE_EXIT &&
        currentState != STATE_RAMP) {
      verifyAndCorrectPosition(sensors.rfidInfo);
    }
  }
  
  if (imuOnline) {
    readIMU(); 
    sensors.compassNorth = getHeadingDegrees();
  }   
}

void readIMU() {
  // Convert gyro angular velocity into yaw by integrating over time. A deadband
  // filters tiny noise around zero after the startup bias calibration.
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

uint32_t runTofMovingAverage(uint32_t sample, uint32_t *history, int &idx, bool &historyReady) {
  // TOF readings can jump between frames, so each distance channel uses a small
  // moving average before the value is used for obstacle, ramp, or rescue logic.
  if (!historyReady) {
    for (int i = 0; i < TOF_FILTER_SIZE; i++) {
      history[i] = sample;
    }
    historyReady = true;
  }

  history[idx] = sample;
  idx = (idx + 1) % TOF_FILTER_SIZE;
  uint64_t sum = 0;
  for (int i = 0; i < TOF_FILTER_SIZE; i++) sum += history[i];
  return sum / TOF_FILTER_SIZE;
}

uint32_t parseTofFrameBytes(uint8_t *frame) {
  // The TOF modules send distance in bytes 8-10 of a validated 16-byte frame.
  return ((uint32_t)frame[8]) | ((uint32_t)frame[9] << 8) | ((uint32_t)frame[10] << 16);
}

bool processTofStream(HardwareSerial &port, uint8_t *frame) {
  // Frame validation protects the robot from acting on partial/noisy UART data:
  // start byte, header bytes, and checksum must all match before accepting it.
  while (port.available()) {
    uint8_t b = port.read();
    if (b != 0x57) continue;
    frame[0] = b;
    unsigned long start = millis();
    int index = 1;
    while (index < 16 && millis() - start < TOF_FRAME_TIMEOUT_MS) {
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
  // Each TOF sensor is read independently so one missing frame does not block
  // the rest of the robot loop. A valid frame marks the channel online; we do
  // not permanently ignore a sensor just because it was quiet during startup.
  if (processTofStream(Serial1, rawFrame1)) {
    tofFrontOnline = true;
    uint32_t raw = parseTofFrameBytes(rawFrame1);
    sensors.tof_front = (int32_t)runTofMovingAverage(raw, tofHistory1, tofIdx1, tofHistoryReady1) - TOF_FRONT_OFFSET;
  }
  if (processTofStream(Serial2, rawFrame2)) {
    tofLeftOnline = true;
    uint32_t raw = parseTofFrameBytes(rawFrame2);
    sensors.tof_left = (int32_t)runTofMovingAverage(raw, tofHistory2, tofIdx2, tofHistoryReady2) - TOF_LEFT_OFFSET;
  }
  if (processTofStream(Serial3, rawFrame3)) {
    tofRight1Online = true;
    uint32_t raw = parseTofFrameBytes(rawFrame3);
    sensors.tof_right1 = (int32_t)runTofMovingAverage(raw, tofHistory3, tofIdx3, tofHistoryReady3) - TOF_RIGHT1_OFFSET;
  }
  if (processTofStream(Serial4, rawFrame4)) {
    tofRight2Online = true;
    uint32_t raw = parseTofFrameBytes(rawFrame4);
    sensors.tof_right2 = (int32_t)runTofMovingAverage(raw, tofHistory4, tofIdx4, tofHistoryReady4) - TOF_RIGHT2_OFFSET;
  }
}

void readIR() {
  // QTR RC reflectance read:
  // 1. Charge each IR sensor pin HIGH.
  // 2. Switch pins back to input.
  // 3. Measure how long each pin takes to discharge.
  // These raw timings are later normalised using sensorMin/sensorMax from
  // IRCalibration.ino before line position is calculated.
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
  // RFID is used as node evidence. A detected tag is packed into sensors.rfidInfo
  // so navigation can confirm intersections and open-field nodes.
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
  // Compass heading is kept as extra telemetry/debug information. The main yaw
  // control uses the calibrated gyro integration above.
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
  

  
//   Serial.print("--- GYRO DRIFT CORRECTED --- ");
//   Serial.print("Old Yaw: "); Serial.print(sensors.yaw);
//   Serial.print(" | Snapped Yaw: "); // Serial.println(snappedYaw);
  
  sensors.yaw = snappedYaw;
}

void DebugSensors() {
  // Viva/demo helper: serial debug mode prints live sensor values so calibration
  // and sensor behaviour can be shown without changing the autonomous code.
  refreshAllSensors();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= DEBUG_PRINT_INTERVAL_MS) {
    lastPrint = millis();
    
    if (debugIRMode) {
      // ==========================================
      // IR ARRAY MODE (Prints all pins)
      // ==========================================
//       Serial.print("IR ARRAY [0-8]: ");
      for (int i = 0; i < IR_COUNT; i++) {
        // Formats it neatly: val, val, val
//         Serial.print(sensors.irLineArray[i]);
//         if (i < IR_COUNT - 1) Serial.print(", "); 
      }
      // Serial.println(); // Drop to next line
    } 
    else {

//       Serial.print("YAW: "); Serial.print(sensors.yaw, 1);
//       Serial.print(" | COMPASS NORTH: "); Serial.print(sensors.compassNorth, 1);
//       Serial.print(" | TOF: F="); Serial.print(sensors.tof_front);
//       Serial.print(", L="); Serial.print(sensors.tof_left);
//       Serial.print(", R1="); Serial.print(sensors.tof_right1);
//       Serial.print(", R2="); Serial.print(sensors.tof_right2);
//       Serial.print(" | IR Center (S5): "); Serial.print(sensors.irLineArray[4]);
//       Serial.print(" | "); 

      static uint32_t lastPrintedID = 0;

      if (sensors.rfidInfo != 0) {
        if (sensors.rfidInfo != lastPrintedID) {
//           Serial.print("Card detected: 0x");
          // Serial.println(sensors.rfidInfo, HEX);
          lastPrintedID = sensors.rfidInfo;
        } else {
          // Serial.println("Card detected: same card");
        }
      } 
      else {
        // Serial.println("no card detected");
        lastPrintedID = 0; 
      }
    }
  }
}
