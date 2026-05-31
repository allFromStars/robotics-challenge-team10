// ============================================================
// Navigation.ino - Line + RFID Node Navigation
// ============================================================

volatile long leftEncoderPos = 0;
volatile long rightEncoderPos = 0;

void readLeftEncoder() {
  if (digitalRead(LEFT_ENCODER_PIN_A) == digitalRead(LEFT_ENCODER_PIN_B)) leftEncoderPos--;
  else leftEncoderPos++;
}

void readRightEncoder() {
  if (digitalRead(RIGHT_ENCODER_PIN_A) == digitalRead(RIGHT_ENCODER_PIN_B)) rightEncoderPos--;
  else rightEncoderPos++;
}

void resetSegmentEncoders() {
  noInterrupts();
  leftEncoderPos = 0;
  rightEncoderPos = 0;
  interrupts();
}

long getAverageEncoderCounts() {
  noInterrupts();
  long leftCounts = abs(leftEncoderPos);
  long rightCounts = abs(rightEncoderPos);
  interrupts();
  return (leftCounts + rightCounts) / 2;
}


// --- Calibration Variables ---
unsigned long calibrationStartMs = 0;
const unsigned long CALIBRATION_TIME_MS = 12000; // 12 seconds of sweeping
bool calibrationComplete = false;




const int GRID_LINE_SPEED = 220;
const int AFTER_RFID_SLOW_SPEED = 150;
const int REACQUIRE_SPEED = 150;
const int MAX_SPEED = 360;

float lineKp = 0.040;
float lineKd = 0.10;
float headingKp = 4.0;
const int HEADING_MAX_CORRECTION = 70;

const long MIN_COUNTS_BEFORE_NODE_CANDIDATE = 250;
const unsigned long NODE_RFID_CONFIRM_TIMEOUT_MS = 1200;
const unsigned long RFID_TO_HOLE_TIMEOUT_MS = 1800;
const unsigned long NODE_PAUSE_MS = 600;
const unsigned long REACQUIRE_AFTER_NODE_TIMEOUT_MS = 1800;
int TARGET_CONFIRMED_NODES = 4;
const unsigned long TASK3_TIMEOUT_MS = 45000;

//IR
uint16_t calibratedValues[IR_COUNT];
uint16_t sensorMin[IR_COUNT] = {0}; // Needs calibration function to populate
uint16_t sensorMax[IR_COUNT] = {1000}; 

const int LINE_CENTER = 4000;
const uint16_t LINE_THRESHOLD = 400;
const uint16_t LINE_TOTAL_THRESHOLD = 300;

uint16_t lastLinePosition = LINE_CENTER;
int lastError = 0;
int activeSensorCount = 0;
bool lineDetected = false;

// --- RFID Tracking Variables ---
uint32_t nodeCandidateTagId = 0;
bool nodeCandidateTagSeen = false;
uint32_t pendingNodeTagId = 0;
bool pendingNodeTag = false;
unsigned long pendingNodeTagSeenMs = 0;


// Call this once to trigger the calibration
void startIRCalibration() {
  Serial.println("--- Starting IR Calibration ---");
  Serial.println("Sweep the sensor array over the white floor and black line!");
  
  // Reset the min and max trackers
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    sensorMin[i] = 2500; // 2500 is the max IR timeout
    sensorMax[i] = 0;
  }
  
  calibrationStartMs = millis();
  calibrationComplete = false;
}

void updateIRCalibration() {
  // Constantly check the live sensor struct to find the new brightest/darkest values
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    if (sensors.irLineArray[i] < sensorMin[i]) {
      sensorMin[i] = sensors.irLineArray[i];
    }
    if (sensors.irLineArray[i] > sensorMax[i]) {
      sensorMax[i] = sensors.irLineArray[i];
    }
  }

  // Print a countdown every 1 second
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    int secondsLeft = (CALIBRATION_TIME_MS - (millis() - calibrationStartMs)) / 1000;
    Serial.print("Calibrating... "); Serial.print(secondsLeft); Serial.println("s remaining");
  }

  // Check if the 12 seconds are up
  if (millis() - calibrationStartMs >= CALIBRATION_TIME_MS) {
    calibrationComplete = true;
    Serial.println("--- IR Calibration Complete! ---");
  }
}

bool isCalibrationComplete() {
  return calibrationComplete;
}




// --- State Machine Enum ---
enum Task3LineState {
  T3_LINE_IDLE, T3_LINE_FOLLOW, T3_NODE_CANDIDATE,
  T3_NODE_CONFIRMED_PAUSE, T3_REACQUIRE_AFTER_NODE,
  T3_LINE_DONE, T3_LINE_FAILSAFE
};
Task3LineState task3LineState = T3_LINE_IDLE;
int confirmedNodes = 0;
unsigned long taskStartMs = 0, stateStartMs = 0, nodeCandidateStartMs = 0;
unsigned long nodePauseStartMs = 0, reacquireStartMs = 0;
float continueHeadingDeg = 0.0;



bool nodeCandidateAllowed() {
  return getAverageEncoderCounts() >= MIN_COUNTS_BEFORE_NODE_CANDIDATE;
}

int clampMagnitude(int speed) {
  if (speed < 0) return 0;
  if (speed > MAX_SPEED) return MAX_SPEED;
  return speed;
}

void driveForwardMagnitudes(int leftMagnitude, int rightMagnitude) {
  leftMagnitude = clampMagnitude(leftMagnitude);
  rightMagnitude = clampMagnitude(rightMagnitude);

  driveMotors(FORWARD_SIGN * leftMagnitude, FORWARD_SIGN * rightMagnitude);
}

float normalizeAngleDeg(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

float headingErrorDeg(float targetDeg, float currentDeg) {
  return normalizeAngleDeg(targetDeg - currentDeg);
}

// --- Sensor Processors ---
void processLinePosition() {
  uint32_t weightedSum = 0;
  uint32_t total = 0;
  activeSensorCount = 0;

  for (uint8_t i = 0; i < IR_COUNT; i++) {
    // Map raw IR data to the calibration logic
    long value = sensors.irLineArray[i]; 
    if (sensorMax[i] > sensorMin[i]) {
        value = ((value - sensorMin[i]) * 1000L) / (sensorMax[i] - sensorMin[i]);
        value = constrain(value, 0, 1000);
    } else {
        value = 0;
    }
    calibratedValues[i] = value;

    if (value >= LINE_THRESHOLD) {
      activeSensorCount++;
      weightedSum += (uint32_t)value * (i * 1000);
      total += value;
    }
  }

  lineDetected = total > LINE_TOTAL_THRESHOLD;
  if (lineDetected) {
    lastLinePosition = weightedSum / total;
  }
}

// ============================================================
// Movement Behaviors
// ============================================================

void followGridLinePD(int baseSpeed) {
  int error = (int)lastLinePosition - LINE_CENTER;
  int derivative = error - lastError;
  int correction = (int)(lineKp * error + lineKd * derivative);

  driveForwardMagnitudes(baseSpeed + correction, baseSpeed - correction);
  lastError = error;
}

void driveForwardWithHeadingHold(float targetHeadingDeg, int baseSpeed) {
  float error = headingErrorDeg(targetHeadingDeg, sensors.yaw); 
  int correction = constrain((int)(headingKp * error), -HEADING_MAX_CORRECTION, HEADING_MAX_CORRECTION);

  driveForwardMagnitudes(baseSpeed - correction, baseSpeed + correction);
}


// Main Loop 


void updateTask3LineNavigation() {

  processLinePosition();
  bool tagDetected = (sensors.rfidInfo != 0); 


  if (taskStartMs > 0 && millis() - taskStartMs > TASK3_TIMEOUT_MS && 
      task3LineState != T3_LINE_IDLE && task3LineState != T3_LINE_DONE && task3LineState != T3_LINE_FAILSAFE) {
    stopMotors();
    task3LineState = T3_LINE_FAILSAFE;
    Serial.println("Task 3 TIMEOUT");
    return;
  }


  switch (task3LineState) {
    case T3_LINE_IDLE:
      stopMotors();
      break;

    case T3_LINE_FOLLOW:
      if (tagDetected && !pendingNodeTag) {
          pendingNodeTagId = sensors.rfidInfo;
          pendingNodeTag = true;
          pendingNodeTagSeenMs = millis();
      }

      if (pendingNodeTag && millis() - pendingNodeTagSeenMs > RFID_TO_HOLE_TIMEOUT_MS) {
        pendingNodeTag = false;
      }

      if (!lineDetected) {
        if (nodeCandidateAllowed()) {
            stopMotors();
            nodeCandidateStartMs = millis();
            continueHeadingDeg = sensors.yaw; 
            nodeCandidateTagId = sensors.rfidInfo;
            nodeCandidateTagSeen = tagDetected;
            
            if (pendingNodeTag) {
                confirmedNodes++;
                nodePauseStartMs = millis();
                nodeCandidateTagSeen = true;
                pendingNodeTag = false;
                task3LineState = T3_NODE_CONFIRMED_PAUSE;
            } else {
                task3LineState = T3_NODE_CANDIDATE;
            }
        } else {
          stopMotors();
          task3LineState = T3_LINE_FAILSAFE;
        }
        break;
      }

      followGridLinePD(pendingNodeTag ? AFTER_RFID_SLOW_SPEED : GRID_LINE_SPEED);
      break;

    case T3_NODE_CANDIDATE:
      stopMotors();
      if (tagDetected) nodeCandidateTagSeen = true;

      if (nodeCandidateTagSeen) {
        confirmedNodes++;
        nodePauseStartMs = millis();
        task3LineState = T3_NODE_CONFIRMED_PAUSE;
      } else if (lineDetected) {
        lastError = 0;
        task3LineState = T3_LINE_FOLLOW;
      } else if (millis() - nodeCandidateStartMs > NODE_RFID_CONFIRM_TIMEOUT_MS) {
        task3LineState = T3_LINE_FAILSAFE;
      }
      break;

    case T3_NODE_CONFIRMED_PAUSE:
      stopMotors();
      if (millis() - nodePauseStartMs >= NODE_PAUSE_MS) {
        if (confirmedNodes >= TARGET_CONFIRMED_NODES) task3LineState = T3_LINE_DONE;
        else {
            resetSegmentEncoders();
            reacquireStartMs = millis();
            lastError = 0;
            pendingNodeTag = false;
            task3LineState = T3_REACQUIRE_AFTER_NODE;
        }
      }
      break;

    case T3_REACQUIRE_AFTER_NODE:
      if (lineDetected) {
        lastError = 0;
        task3LineState = T3_LINE_FOLLOW;
      } else {
        driveForwardWithHeadingHold(continueHeadingDeg, REACQUIRE_SPEED);
        if (millis() - reacquireStartMs > REACQUIRE_AFTER_NODE_TIMEOUT_MS) task3LineState = T3_LINE_FAILSAFE;
      }
      break;

    case T3_LINE_DONE:
    case T3_LINE_FAILSAFE:
      stopMotors();
      break;
  }
}


void startTask3LineNavigation() {
  confirmedNodes = 0;
  taskStartMs = millis();
  pendingNodeTag = false;
  lastError = 0;
  resetSegmentEncoders();
  task3LineState = T3_LINE_FOLLOW;
}


bool task3LineNavigationComplete() {
  return task3LineState == T3_LINE_DONE;
}

bool task3LineNavigationFailed() {
  return task3LineState == T3_LINE_FAILSAFE;
}