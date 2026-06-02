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


// --- State Machine Enum ---
enum Task3LineState {
  T3_LINE_IDLE, T3_LEAVE_START_NODE, T3_LINE_FOLLOW, T3_NODE_CANDIDATE,
  T3_NODE_CONFIRMED_PAUSE, T3_ADVANCE_TO_TURN_CENTER, T3_REACQUIRE_AFTER_NODE,
  T3_LINE_DONE, T3_LINE_FAILSAFE
};
Task3LineState task3LineState = T3_LINE_IDLE;
unsigned long taskStartMs = 0, nodeCandidateStartMs = 0;
unsigned long nodePauseStartMs = 0, nodeAdvanceStartMs = 0, reacquireStartMs = 0;
float continueHeadingDeg = 0.0;
LineArrivalMode lineArrivalMode = LINE_ARRIVE_FOR_TURN;
const char* task3FailureReason = "";



bool nodeCandidateAllowed() {
  return getAverageEncoderCounts() >= MIN_COUNTS_BEFORE_NODE_CANDIDATE;
}

int clampMagnitude(int speed) {
  if (speed < 0) return 0;
  if (speed > LINE_NAV_MAX_SPEED) return LINE_NAV_MAX_SPEED;
  return speed;
}

void driveForwardMagnitudes(int leftMagnitude, int rightMagnitude) {
  leftMagnitude = clampMagnitude(leftMagnitude);
  rightMagnitude = clampMagnitude(rightMagnitude);

  driveMotors(FORWARD_SIGN * leftMagnitude, FORWARD_SIGN * rightMagnitude);
}

float normaliseAngleDeg(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

float headingErrorDeg(float targetDeg, float currentDeg) {
  return normaliseAngleDeg(targetDeg - currentDeg);
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

void driveForwardWithHeadingHold(float targetHeadingDeg, int baseSpeed, float kp, int maxCorrection) {
  float error = headingErrorDeg(targetHeadingDeg, sensors.yaw); 
  int correction = constrain((int)(kp * error), -maxCorrection, maxCorrection);

  driveForwardMagnitudes(baseSpeed - correction, baseSpeed + correction);
}

void driveForwardWithHeadingHold(float targetHeadingDeg, int baseSpeed) {
  driveForwardWithHeadingHold(targetHeadingDeg, baseSpeed, headingKp, HEADING_MAX_CORRECTION);
}


// Main Loop 


void updateTask3LineNavigation() {

  processLinePosition();
  bool tagDetected = (sensors.rfidInfo != 0); 


  if (taskStartMs > 0 && millis() - taskStartMs > LINE_NAV_TIMEOUT_MS && 
      task3LineState != T3_LINE_IDLE && task3LineState != T3_LINE_DONE && task3LineState != T3_LINE_FAILSAFE) {
    stopMotors();
    task3FailureReason = "line navigation timeout";
    task3LineState = T3_LINE_FAILSAFE;
    Serial.println("Line navigation TIMEOUT");
    return;
  }


  switch (task3LineState) {
    case T3_LINE_IDLE:
      stopMotors();
      break;

    case T3_LEAVE_START_NODE:
      if (lineDetected) {
        lastError = 0;
        resetSegmentEncoders();
        task3LineState = T3_LINE_FOLLOW;
      } else {
        driveForwardWithHeadingHold(continueHeadingDeg, REACQUIRE_SPEED);
        if (millis() - reacquireStartMs > START_NODE_EXIT_TIMEOUT_MS) {
          task3FailureReason = "could not find line after leaving start node";
          task3LineState = T3_LINE_FAILSAFE;
        }
      }
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
                nodePauseStartMs = millis();
                nodeCandidateTagSeen = true;
                pendingNodeTag = false;
                task3LineState = T3_NODE_CONFIRMED_PAUSE;
            } else {
                task3LineState = T3_NODE_CANDIDATE;
            }
        } else {
          continueHeadingDeg = sensors.yaw;
          reacquireStartMs = millis();
          task3LineState = T3_REACQUIRE_AFTER_NODE;
        }
        break;
      }

      followGridLinePD(pendingNodeTag ? AFTER_RFID_SLOW_SPEED : GRID_LINE_SPEED);
      break;

    case T3_NODE_CANDIDATE:
      stopMotors();
      if (tagDetected) nodeCandidateTagSeen = true;

      if (nodeCandidateTagSeen) {
        nodePauseStartMs = millis();
        task3LineState = T3_NODE_CONFIRMED_PAUSE;
      } else if (lineDetected) {
        lastError = 0;
        task3LineState = T3_LINE_FOLLOW;
      } else if (millis() - nodeCandidateStartMs > NODE_CANDIDATE_CONFIRM_MS) {
        nodePauseStartMs = millis();
        task3LineState = T3_NODE_CONFIRMED_PAUSE;
      }
      break;

    case T3_NODE_CONFIRMED_PAUSE:
      stopMotors();
      if (millis() - nodePauseStartMs >= NODE_PAUSE_MS) {
        if (lineArrivalMode == LINE_ARRIVE_FOR_PLANTING) {
          task3LineState = T3_LINE_DONE;
        } else {
          resetSegmentEncoders();
          nodeAdvanceStartMs = millis();
          task3LineState = T3_ADVANCE_TO_TURN_CENTER;
        }
      }
      break;

    case T3_ADVANCE_TO_TURN_CENTER:
      if (lineDetected) {
        followGridLinePD(TURN_CENTER_ADVANCE_SPEED);
      } else {
        lastError = 0;
        driveForwardWithHeadingHold(continueHeadingDeg, TURN_CENTER_ADVANCE_SPEED);
      }

      if (getAverageEncoderCounts() >= TURN_CENTER_ADVANCE_COUNTS) {
        stopMotors();
        task3LineState = T3_LINE_DONE;
      } else if (millis() - nodeAdvanceStartMs > TURN_CENTER_ADVANCE_TIMEOUT_MS) {
        stopMotors();
        Serial.println("Turn-center advance timeout; continuing to plan.");
        task3LineState = T3_LINE_DONE;
      }
      break;

    case T3_REACQUIRE_AFTER_NODE:
      if (lineDetected) {
        lastError = 0;
        resetSegmentEncoders();
        task3LineState = T3_LINE_FOLLOW;
      } else {
        driveForwardWithHeadingHold(continueHeadingDeg, REACQUIRE_SPEED);
        if (millis() - reacquireStartMs > REACQUIRE_AFTER_NODE_TIMEOUT_MS) {
          task3FailureReason = "could not reacquire line";
          task3LineState = T3_LINE_FAILSAFE;
        }
      }
      break;

    case T3_LINE_DONE:
    case T3_LINE_FAILSAFE:
      stopMotors();
      break;
  }
}


void startTask3LineNavigation(LineArrivalMode arrivalMode) {
  taskStartMs = millis();
  pendingNodeTag = false;
  lastError = 0;
  lineArrivalMode = arrivalMode;
  continueHeadingDeg = sensors.yaw;
  reacquireStartMs = millis();
  resetSegmentEncoders();
  task3LineState = T3_LEAVE_START_NODE;
}

void startTask3LineNavigation() {
  startTask3LineNavigation(LINE_ARRIVE_FOR_TURN);
}


bool task3LineNavigationComplete() {
  return task3LineState == T3_LINE_DONE;
}

bool task3LineNavigationFailed() {
  return task3LineState == T3_LINE_FAILSAFE;
}

const char* getTask3LineNavigationFailureReason() {
  return task3FailureReason;
}

// ============================================================
// Task 4 - Open-field one-edge navigation
// ============================================================

enum Task4OpenState {
  T4_OPEN_IDLE,
  T4_OPEN_DRIVE_EDGE,
  T4_OPEN_SEARCH_NODE,
  T4_OPEN_NODE_PAUSE,
  T4_OPEN_DONE,
  T4_OPEN_FAILSAFE
};

Task4OpenState task4OpenState = T4_OPEN_IDLE;

unsigned long openTaskStartMs = 0;
unsigned long openSearchStartMs = 0;
unsigned long openNodePauseStartMs = 0;
float openTargetHeadingDeg = 0.0;
uint32_t openConfirmedTagId = 0;

bool nearExpectedOpenNode() {
  long searchStartCounts = OPEN_GRID_EDGE_COUNTS - OPEN_NODE_SEARCH_WINDOW_COUNTS;

  if (searchStartCounts < 0) {
    searchStartCounts = 0;
  }

  return getAverageEncoderCounts() >= searchStartCounts;
}

bool openEdgeDistanceReached() {
  return getAverageEncoderCounts() >= OPEN_GRID_EDGE_COUNTS;
}

void driveOpenWithHeadingHold(int baseSpeed) {
  driveForwardWithHeadingHold(
    openTargetHeadingDeg,
    baseSpeed,
    OPEN_HEADING_KP,
    OPEN_HEADING_MAX_CORRECTION
  );
}

void startTask4OpenNavigation() {
  openTaskStartMs = millis();
  openSearchStartMs = 0;
  openNodePauseStartMs = 0;
  openTargetHeadingDeg = sensors.yaw;
  openConfirmedTagId = 0;
  resetSegmentEncoders();
  task4OpenState = T4_OPEN_DRIVE_EDGE;

  Serial.print("Task 4 open edge start. targetHeading=");
  Serial.println(openTargetHeadingDeg, 1);
}

void updateTask4OpenNavigation() {
  bool tagDetected = sensors.rfidInfo != 0;

  if (openTaskStartMs > 0 &&
      millis() - openTaskStartMs > OPEN_EDGE_TIMEOUT_MS &&
      task4OpenState != T4_OPEN_IDLE &&
      task4OpenState != T4_OPEN_DONE &&
      task4OpenState != T4_OPEN_FAILSAFE) {
    stopMotors();
    task4OpenState = T4_OPEN_FAILSAFE;
    Serial.println("Open navigation TIMEOUT");
    return;
  }

  switch (task4OpenState) {
    case T4_OPEN_IDLE:
      stopMotors();
      break;

    case T4_OPEN_DRIVE_EDGE:
      if (nearExpectedOpenNode()) {
        openSearchStartMs = millis();
        task4OpenState = T4_OPEN_SEARCH_NODE;
        Serial.println("Open edge near target. Searching for RFID node.");
        break;
      }

      driveOpenWithHeadingHold(OPEN_FIELD_SPEED);
      break;

    case T4_OPEN_SEARCH_NODE:
      if (tagDetected) {
        openConfirmedTagId = sensors.rfidInfo;
        openNodePauseStartMs = millis();
        stopMotors();

        Serial.print("Open node RFID confirmed: 0x");
        Serial.println(openConfirmedTagId, HEX);

        task4OpenState = T4_OPEN_NODE_PAUSE;
        break;
      }

      if (millis() - openSearchStartMs > OPEN_NODE_SEARCH_TIMEOUT_MS) {
        stopMotors();
        task4OpenState = T4_OPEN_FAILSAFE;
        Serial.println("Open navigation failed: RFID node not found near target");
        break;
      }

      driveOpenWithHeadingHold(OPEN_SEARCH_SPEED);
      break;

    case T4_OPEN_NODE_PAUSE:
      stopMotors();

      if (millis() - openNodePauseStartMs >= OPEN_NODE_PAUSE_MS) {
        task4OpenState = T4_OPEN_DONE;
      }
      break;

    case T4_OPEN_DONE:
    case T4_OPEN_FAILSAFE:
      stopMotors();
      break;
  }
}

bool task4OpenNavigationComplete() {
  return task4OpenState == T4_OPEN_DONE;
}

bool task4OpenNavigationFailed() {
  return task4OpenState == T4_OPEN_FAILSAFE;
}

// ============================================================
// Post-planting advance
// ============================================================

bool postPlantAdvanceActive = false;
unsigned long postPlantAdvanceStartMs = 0;
float postPlantAdvanceHeadingDeg = 0.0;

void startPostPlantAdvance() {
  postPlantAdvanceActive = true;
  postPlantAdvanceStartMs = millis();
  postPlantAdvanceHeadingDeg = sensors.yaw;
  resetSegmentEncoders();
}

bool updatePostPlantAdvance() {
  if (!postPlantAdvanceActive) return true;

  processLinePosition();

  if (lineDetected) {
    followGridLinePD(POST_PLANT_ADVANCE_SPEED);
  } else {
    lastError = 0;
    driveForwardWithHeadingHold(postPlantAdvanceHeadingDeg, POST_PLANT_ADVANCE_SPEED);
  }

  if (getAverageEncoderCounts() >= POST_PLANT_ADVANCE_COUNTS) {
    stopMotors();
    postPlantAdvanceActive = false;
    return true;
  }

  if (millis() - postPlantAdvanceStartMs > POST_PLANT_ADVANCE_TIMEOUT_MS) {
    stopMotors();
    postPlantAdvanceActive = false;
    Serial.println("Post-plant advance timeout; continuing to plan.");
    return true;
  }

  return false;
}


// Ramp Wall Following with distance sensors + front distance sensors for stopping

static bool rampInitialised = false;
static float wallLastError = 0;
static float speedIntegral = 0;
static unsigned long lastWallMicros = 0;
static long lastAvgEncoderCount = 0;

void startRampWallFollowing() {
  Serial.println("[RAMP] Initialising Wall Follower & Clearing Memory...");
  wallLastError = 0;
  speedIntegral = 0;
  resetSegmentEncoders();
  lastAvgEncoderCount = 0;
  lastWallMicros = micros();
  rampInitialised = true;
}

// Returns 1 if RFID reached, 0 if still driving/waiting
int updateRampWallFollowing(bool trackLeftWall) {
  
  // initialise
  if (!rampInitialised || (micros() - lastWallMicros > 200000)) {
    startRampWallFollowing();
  }

  // check for rfid to finish
  if (sensors.rfidInfo != 0) {
    stopMotors();
    rampInitialised = false; // Reset for next time
    return 1; // Signal completion to Main loop
  }


  unsigned long currentMicros = micros();
  if (currentMicros - lastWallMicros < 1000) return 0;
  float dt = (currentMicros - lastWallMicros) / 1000000.0;
  lastWallMicros = currentMicros;

  // TAILGATING / stopping
  float dynamicTargetSpeed = TARGET_ENCODER_SPEED; 

  if (sensors.tof_front > 0) {
    if (sensors.tof_front < FRONT_COLLISION_DIST_MM) {
      stopMotors();
      speedIntegral = 0; 
      lastAvgEncoderCount = getAverageEncoderCounts(); 
      
      static unsigned long lastWaitPrint = 0;
      if (millis() - lastWaitPrint > 1000) {
        Serial.println("[RAMP] Obstacle critically close. Holding position...");
        lastWaitPrint = millis();
      }
      return 0; // Stay parked
    } 
    else if (sensors.tof_front < TAILGATE_START_DIST_MM) {
      float distanceRatio = (float)(sensors.tof_front - FRONT_COLLISION_DIST_MM) / 
                            (float)(TAILGATE_START_DIST_MM - FRONT_COLLISION_DIST_MM);
      dynamicTargetSpeed = TARGET_ENCODER_SPEED * distanceRatio;
      if (dynamicTargetSpeed < 250.0) dynamicTargetSpeed = 250.0; 
    }
  }

  // TORQUE
  long currentCounts = getAverageEncoderCounts();
  float currentSpeed = (currentCounts - lastAvgEncoderCount) / dt;
  lastAvgEncoderCount = currentCounts;

  float speedError = dynamicTargetSpeed - currentSpeed;
  speedIntegral += speedError * dt;
  
  if (speedIntegral > 300) speedIntegral = 300;
  if (speedIntegral < -300) speedIntegral = -300;

  int basePWM = (int)(speedError * speedKp + speedIntegral * speedKi);
  

  // allow motors to apply up to -200 reverse PWM to actively brake if going too fast (decline)
  if (basePWM < -200) basePWM = -200; 
  if (basePWM > RAMP_MAX_PWM) basePWM = RAMP_MAX_PWM;

  // STEERING 
  int currentWallDist = trackLeftWall ? sensors.tof_left : sensors.tof_right1;
  int correction = 0;
  
  if (currentWallDist > 0 && currentWallDist < 600) {
    float error = TARGET_WALL_DIST_MM - currentWallDist;
    float derivative = (error - wallLastError) / dt;
    wallLastError = error;

    correction = (int)(wallKp * error + wallKd * derivative);
    correction = constrain(correction, -WALL_MAX_CORRECTION, WALL_MAX_CORRECTION);
  }


  // Positive correction means the robot is too close to the tracked wall.
  // Apply steering away from that wall: away from left wall = turn right,
  // away from right wall = turn left.
  int leftSpeed = basePWM - correction;
  int rightSpeed = basePWM + correction;

  if (!trackLeftWall) {
     leftSpeed = basePWM + correction;
     rightSpeed = basePWM - correction;
  }

  driveMotors(leftSpeed, rightSpeed);
  return 0; // Still driving
}


// Touch-Based Robot Revival

static bool rescueInitialised = false;
static unsigned long rescueStartMs = 0; 
static float rescueTargetHeading = 0.0; // remember heading incase no lines

void startRescueApproach() {
  Serial.println("[RESCUE] Target locked. Initiating approach vector...");
  rescueInitialised = true;
  lastError = 0; 
  resetSegmentEncoders();
  rescueStartMs = millis(); 
  
  // get IMU angle and lock
  rescueTargetHeading = sensors.yaw; 
}

// Returns 1 if successful, 0 if approaching, -1 if timeout/failed
int updateRescueApproach() {
  if (!rescueInitialised) startRescueApproach();

  // Timeout Check

  if (millis() - rescueStartMs > RESCUE_TIMEOUT_MS) {
    stopMotors();
    rescueInitialised = false;
    Serial.println("[RESCUE ERROR] Timeout! Target missed or switch failed.");
    return -1; // Signal a failure to the main loop
  }

  int dist = sensors.tof_front;

  // Physical Switch (D13)
  if (digitalRead(BUMPER_PIN) == LOW) {
    stopMotors();
    rescueInitialised = false; 
    Serial.println("[RESCUE] Mechanical Contact Confirmed!");
    return 1; 
  }

  // TOF Proportional Velocity Ramp
  int targetSpeed = RESCUE_MAX_SPEED;
  if (dist > 0 && dist < RESCUE_DETECT_DIST_MM) {
    float distanceRatio = 0.0;
    if (dist > RESCUE_CONTACT_DIST_MM) {
      distanceRatio = (float)(dist - RESCUE_CONTACT_DIST_MM) / 
                      (float)(RESCUE_DETECT_DIST_MM - RESCUE_CONTACT_DIST_MM);
    }
    targetSpeed = RESCUE_MIN_SPEED + (int)(distanceRatio * (RESCUE_MAX_SPEED - RESCUE_MIN_SPEED));
  }

  processLinePosition();

  if (lineDetected) {
    // If we have lines, use the IR sensors
    followGridLinePD(targetSpeed);
  } else {
    // If there is no tape, use IMU angle
    driveForwardWithHeadingHold(rescueTargetHeading, targetSpeed);
  }

  return 0; // Still approaching
}
