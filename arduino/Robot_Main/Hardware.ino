#include <Servo.h>


void initWire() {
  // Serial.println("Initialising wire...");
  Wire.begin();
  Wire.setClock(400000);
  Wire1.begin();
  Wire1.setClock(400000); 
  delay(100); 
}

bool initMotors() {
  if (!checkI2CDevice(Wire1, 16)) {
    return false;
  }

  mc.setBus(&Wire1);
  mc.reinitialize();
  mc.clearResetFlag();
  mc.setCommandTimeoutMilliseconds(MOTOR_COMMAND_TIMEOUT_MS);
  
  mc.setMaxAcceleration(LEFT_MOTOR_CHANNEL, MAX_ACCEL);
  mc.setMaxDeceleration(LEFT_MOTOR_CHANNEL, MAX_DECEL);
  mc.setMaxAcceleration(RIGHT_MOTOR_CHANNEL, MAX_ACCEL);
  mc.setMaxDeceleration(RIGHT_MOTOR_CHANNEL, MAX_DECEL);
  
  
  stopMotors();
  return true;
}

void driveMotors(int leftLogicalSpeed, int rightLogicalSpeed) {
  if (motorOnline) {
    // keeps speed between -600 and 600
    leftLogicalSpeed = constrain(leftLogicalSpeed, -MaxMotorSpeed, MaxMotorSpeed);
    rightLogicalSpeed = constrain(rightLogicalSpeed, -MaxMotorSpeed, MaxMotorSpeed);

    mc.setSpeed(LEFT_MOTOR_CHANNEL, LEFT_DIR * leftLogicalSpeed);
    mc.setSpeed(RIGHT_MOTOR_CHANNEL, RIGHT_DIR * rightLogicalSpeed);
  }
}

void stopMotors() {
  if (motorOnline) {
    mc.setSpeed(LEFT_MOTOR_CHANNEL, 0);
    mc.setSpeed(RIGHT_MOTOR_CHANNEL, 0);
  }
}



static bool isTurning = false;
static float targetYaw = 0;
static float turnIntegral = 0;
static float lastTurnError = 0;
static unsigned long settleStartTime = 0;
static unsigned long lastPidMicros = 0; 
static unsigned long lastTurnDebugPrintMs = 0;

void startTurnAngle(float targetAngle) {
//   Serial.print("Initiating Pivot Turn to: "); 
//   Serial.print(targetAngle); 
  // Serial.println("°");
  

  targetYaw = sensors.yaw + targetAngle; 
  
  turnIntegral = 0;
  lastTurnError = targetAngle;
  settleStartTime = 0;
  lastPidMicros = micros();
  lastTurnDebugPrintMs = 0;
  isTurning = true;
}

void cancelTurnAngle() {
  stopMotors();
  isTurning = false;
  targetYaw = sensors.yaw;
  turnIntegral = 0;
  lastTurnError = 0;
  settleStartTime = 0;
  lastPidMicros = micros();
}

bool updateTurnAngle() {
  if (!isTurning) return true; 

  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastPidMicros) / 1000000.0;
  lastPidMicros = currentMicros;

  if (dt <= 0.0) {
    dt = 0.001;
  }

  float error = targetYaw - sensors.yaw;

  if (abs(error) <= TURN_ANGLE_TOLERANCE_DEG) {
    if (settleStartTime == 0) {
      settleStartTime = millis();
    }
    if (millis() - settleStartTime >= TURN_SETTLE_MS) {
      stopMotors();
      isTurning = false;
      // Serial.println("Turn Complete.");
      return true; 
    }
  } else {
    settleStartTime = 0; 
  }

  turnIntegral += error * dt;
  turnIntegral = constrain(turnIntegral, -TURN_INTEGRAL_LIMIT, TURN_INTEGRAL_LIMIT);

  float derivative = (error - lastTurnError) / dt;
  lastTurnError = error;

  float totalOutput = turnKp * error + turnKi * turnIntegral + turnKd * derivative;

  if (abs(totalOutput) < TURN_MOTOR_MIN_LIMIT && abs(error) > TURN_ANGLE_TOLERANCE_DEG) {
    totalOutput = (totalOutput > 0.0) ? TURN_MOTOR_MIN_LIMIT : -TURN_MOTOR_MIN_LIMIT;
  }

  totalOutput = constrain(totalOutput, -TURN_MOTOR_MAX_LIMIT, TURN_MOTOR_MAX_LIMIT);

  int leftSpeed  = FORWARD_SIGN * totalOutput; 
  int rightSpeed = -FORWARD_SIGN * totalOutput;

  driveMotors(leftSpeed, rightSpeed);

  if (millis() - lastTurnDebugPrintMs >= TURN_DEBUG_PRINT_INTERVAL_MS) {
    lastTurnDebugPrintMs = millis();
//     Serial.print("[TURN] targetYaw=");
//     Serial.print(targetYaw, 1);
//     Serial.print(" yaw=");
//     Serial.print(sensors.yaw, 1);
//     Serial.print(" error=");
//     Serial.print(error, 1);
//     Serial.print(" gyroBiasZ=");
//     Serial.print(gyroBiasZ, 4);
//     Serial.print(" output=");
//     Serial.print(totalOutput, 1);
//     Serial.print(" L=");
//     Serial.print(leftSpeed);
//     Serial.print(" R=");
    // Serial.println(rightSpeed);
  }

  return false; 
}


static Servo planterServo;

// --- Planter Sub-State Machine ---
enum PlanterState { P_IDLE, P_MOVING_OUT, P_HOLDING, P_RETURNING };
static PlanterState currentPlanterState = P_IDLE;

static unsigned long planterTimer = 0;
static int currentAngle = 0;
static int plantAngle = 30;


// call once to start
void startPlanting() {
  // Serial.println("Starting planting sequence...");
  planterServo.attach(SERVO_PIN);
  planterServo.write(0);
  currentAngle = 0;
  planterTimer = millis();
  currentPlanterState = P_MOVING_OUT;
}


// Returns 'true' when the seed is fully planted and the arm has returned.
bool updatePlanting() {
  unsigned long currentMillis = millis();

  switch (currentPlanterState) {
    case P_IDLE:

      return true; 

    case P_MOVING_OUT:
      if (currentMillis - planterTimer >= 50) {
        planterTimer = currentMillis;
        currentAngle++;
        planterServo.write(currentAngle);
        
        if (currentAngle >= plantAngle) {
          planterServo.detach(); // stop incase of stall
          currentPlanterState = P_HOLDING;
          robotInfo.seedsLeft--;
        }
      }
      break;

    case P_HOLDING:
      
      if (currentMillis - planterTimer >= 2000) {
        planterServo.attach(SERVO_PIN);
        planterTimer = currentMillis;
        currentPlanterState = P_RETURNING;

      }
      break;

    case P_RETURNING:
      if (currentMillis - planterTimer >= 50) {
        planterTimer = currentMillis;
        currentAngle--;
        planterServo.write(currentAngle);
        
        if (currentAngle <= 0) {
          planterServo.detach();
          currentPlanterState = P_IDLE; // Reset for next time
          // Serial.println("Planting complete! Arm returned.");
          return true; 
        }
      }
      break;
  }
  
  return false; 
}


void abortPlanting() {
  if (currentPlanterState != P_IDLE) {
    if (!planterServo.attached()) planterServo.attach(SERVO_PIN);
    planterServo.write(0); 
    delay(300); 
    planterServo.detach();
    currentPlanterState = P_IDLE;
  }
}
