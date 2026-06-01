#include <Servo.h>


void initWire() {
  Serial.println("Initialising wire...");
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

void startTurnAngle(float targetAngle) {
  Serial.print("Initiating Pivot Turn to: "); 
  Serial.print(targetAngle); 
  Serial.println("°");
  

  targetYaw = sensors.yaw + targetAngle; 
  
  turnIntegral = 0;
  lastTurnError = targetAngle;
  settleStartTime = 0;
  lastPidMicros = micros();
  isTurning = true;
}

bool updateTurnAngle() {
  if (!isTurning) return true; 

  unsigned long currentMicros = micros();
  

  if (currentMicros - lastPidMicros < 1000) return false; 
  float dt = (currentMicros - lastPidMicros) / 1000000.0;
  lastPidMicros = currentMicros;


  float error = targetYaw - sensors.yaw;
  

  float P_out = error * turnKp;

  turnIntegral += error * dt;
  if (turnIntegral > 1000)  turnIntegral = 1000;
  if (turnIntegral < -1000) turnIntegral = -1000;
  float I_out = turnIntegral * turnKi;

  float D_out = ((error - lastTurnError) / dt) * turnKd;
  lastTurnError = error;

  float totalOutput = P_out + I_out + D_out;


  if (abs(totalOutput) < MOTOR_MIN_LIMIT && abs(error) > 0.8) {
    totalOutput = (totalOutput > 0) ? MOTOR_MIN_LIMIT : -MOTOR_MIN_LIMIT;
  }


  if (totalOutput > MOTOR_MAX_LIMIT)  totalOutput = MOTOR_MAX_LIMIT;
  if (totalOutput < -MOTOR_MAX_LIMIT) totalOutput = -MOTOR_MAX_LIMIT;

  int leftSpeed  = FORWARD_SIGN * totalOutput; 
  int rightSpeed = -FORWARD_SIGN * totalOutput;

  driveMotors(leftSpeed, rightSpeed);

  // check Complete
  if (abs(error) <= 1.0) {
    if (settleStartTime == 0) {
      settleStartTime = millis();
    }
    if (millis() - settleStartTime >= 80) {
      stopMotors();
      isTurning = false;
      Serial.println("Turn Complete.");
      return true; 
    }
  } else {
    settleStartTime = 0; 
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
  Serial.println("Starting planting sequence...");
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
          Serial.println("Planting complete! Arm returned.");
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