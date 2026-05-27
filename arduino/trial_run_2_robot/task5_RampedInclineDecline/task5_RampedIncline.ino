#include <Wire.h>
#include <Motoron.h>

// Motoron controller address
MotoronI2C motorController(16);

// Motor channels
const int leftMotorChannel = 1;
const int rightMotorChannel = 2;

// Motor direction correction
// One motor is mounted opposite, so the right side is reversed in software
const int leftMotorDirection = 1;
const int rightMotorDirection = -1;

// Encoder pins
const int leftEncoderA = D2;
const int leftEncoderB = D3;

const int rightEncoderA = D4;
const int rightEncoderB = D5;

// Encoder count variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// LED used to show when the test has stopped/finished
const int statusLed = D9;

// The ramp task has to finish within 45 seconds
const unsigned long rampTimeLimit = 45000;

// This is the distance the robot should travel to finish the ramp
// This value will need tuning on the actual ramp
const long rampDistanceCounts = 2500;

// PID is updated every 50 ms
const unsigned long pidUpdateTime = 50;

// Target wheel speed while climbing the ramp
// Unit is encoder counts per second
const float targetClimbSpeed = 350.0;

// Starting motor power for climbing the ramp
// Negative speed moves our robot forward
const int climbBaseSpeed = -300;

// Motor speed limits
const int minimumClimbSpeed = 180;
const int maximumClimbSpeed = 500;

// PID tuning values
float climbKp = 0.45;
float climbKi = 0.02;
float climbKd = 0.08;

// PID memory for the left motor
float leftTotalError = 0;
float leftPreviousError = 0;

// PID memory for the right motor
float rightTotalError = 0;
float rightPreviousError = 0;

// Used to work out wheel speed from encoder counts
long previousLeftCount = 0;
long previousRightCount = 0;

unsigned long previousPidTime = 0;
unsigned long rampStartTime = 0;
unsigned long previousPrintTime = 0;

int currentLeftMotorSpeed = 0;
int currentRightMotorSpeed = 0;

bool rampHasStarted = false;
bool rampComplete = false;
bool rampFailed = false;

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(10);
  }

  Serial.println();
  Serial.println("Ramp incline test starting...");
  Serial.println("Robot will start climbing after setup.");
  Serial.println();

  Wire1.begin();

  motorController.setBus(&Wire1);
  motorController.reinitialize();
  motorController.clearResetFlag();

  motorController.setCommandTimeoutMilliseconds(1000);

  // Keep acceleration smooth so the robot does not jerk on the ramp
  motorController.setMaxAcceleration(leftMotorChannel, 120);
  motorController.setMaxDeceleration(leftMotorChannel, 350);

  motorController.setMaxAcceleration(rightMotorChannel, 120);
  motorController.setMaxDeceleration(rightMotorChannel, 350);

  pinMode(leftEncoderA, INPUT_PULLUP);
  pinMode(leftEncoderB, INPUT_PULLUP);

  pinMode(rightEncoderA, INPUT_PULLUP);
  pinMode(rightEncoderB, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(leftEncoderA),
    updateLeftEncoder,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(rightEncoderA),
    updateRightEncoder,
    CHANGE
  );

  pinMode(statusLed, OUTPUT);

  stopRobot();

  delay(2000);

  startRampClimb();
}

void loop() {
  motorController.resetCommandTimeout();

  AscendRamp();

  printRampStatus();
}

void startRampClimb() {
  resetEncoders();
  resetPidValues();

  rampStartTime = millis();
  previousPidTime = millis();

  rampHasStarted = true;
  rampComplete = false;
  rampFailed = false;

  digitalWrite(statusLed, LOW);

  Serial.println("Ramp climb started.");
}

void AscendRamp() {
  if (!rampHasStarted || rampComplete || rampFailed) {
    stopRobot();
    return;
  }

  unsigned long timeUsed = millis() - rampStartTime;

  long leftCounts;
  long rightCounts;

  getEncoderCounts(leftCounts, rightCounts);

  long averageDistance = (leftCounts + rightCounts) / 2;

  // Stop when the robot has travelled far enough to finish the ramp
  if (averageDistance >= rampDistanceCounts) {
    stopRobot();

    rampComplete = true;
    digitalWrite(statusLed, HIGH);

    Serial.println();
    Serial.println("RAMP CLIMB COMPLETE");
    Serial.println("Robot reached the set ramp distance.");
    Serial.println();

    return;
  }

  // Stop if the robot takes too long
  if (timeUsed >= rampTimeLimit) {
    stopRobot();

    rampFailed = true;
    digitalWrite(statusLed, HIGH);

    Serial.println();
    Serial.println("RAMP CLIMB FAILED");
    Serial.println("Robot did not finish within 45 seconds.");
    Serial.println();

    return;
  }

  unsigned long currentTime = millis();

  if (currentTime - previousPidTime >= pidUpdateTime) {
    float timeStep = (currentTime - previousPidTime) / 1000.0;

    long leftChange = leftCounts - previousLeftCount;
    long rightChange = rightCounts - previousRightCount;

    float leftWheelSpeed = leftChange / timeStep;
    float rightWheelSpeed = rightChange / timeStep;

    int leftMotorCommand = calculateClimbSpeed(
      targetClimbSpeed,
      leftWheelSpeed,
      leftTotalError,
      leftPreviousError,
      timeStep
    );

    int rightMotorCommand = calculateClimbSpeed(
      targetClimbSpeed,
      rightWheelSpeed,
      rightTotalError,
      rightPreviousError,
      timeStep
    );

    currentLeftMotorSpeed = leftMotorCommand;
    currentRightMotorSpeed = rightMotorCommand;

    driveRobot(currentLeftMotorSpeed, currentRightMotorSpeed);

    previousLeftCount = leftCounts;
    previousRightCount = rightCounts;
    previousPidTime = currentTime;
  }
}

int calculateClimbSpeed(float targetSpeed, float measuredSpeed, float &totalError, float &previousError, float timeStep) {
  float error = targetSpeed - measuredSpeed;

  totalError += error * timeStep;

  // Limit the total error so the integral part does not build up too much
  if (totalError > 500) {
    totalError = 500;
  }

  if (totalError < -500) {
    totalError = -500;
  }

  float errorChange = (error - previousError) / timeStep;

  float pidOutput = (climbKp * error) + (climbKi * totalError) + (climbKd * errorChange);

  previousError = error;

  int speedAmount = abs(climbBaseSpeed) + pidOutput;

  if (speedAmount > maximumClimbSpeed) {
    speedAmount = maximumClimbSpeed;
  }

  if (speedAmount < minimumClimbSpeed) {
    speedAmount = minimumClimbSpeed;
  }

  // Negative means forward for our robot
  return -speedAmount;
}

void driveRobot(int leftSpeed, int rightSpeed) {
  motorController.setSpeed(
    leftMotorChannel,
    leftMotorDirection * leftSpeed
  );

  motorController.setSpeed(
    rightMotorChannel,
    rightMotorDirection * rightSpeed
  );
}

void stopRobot() {
  motorController.setSpeed(leftMotorChannel, 0);
  motorController.setSpeed(rightMotorChannel, 0);

  currentLeftMotorSpeed = 0;
  currentRightMotorSpeed = 0;
}

void resetEncoders() {
  noInterrupts();

  leftEncoderCount = 0;
  rightEncoderCount = 0;

  interrupts();

  previousLeftCount = 0;
  previousRightCount = 0;
}

void resetPidValues() {
  leftTotalError = 0;
  rightTotalError = 0;

  leftPreviousError = 0;
  rightPreviousError = 0;
}

void getEncoderCounts(long &leftCounts, long &rightCounts) {
  noInterrupts();

  long leftRaw = leftEncoderCount;
  long rightRaw = rightEncoderCount;

  interrupts();

  leftCounts = abs(leftRaw);
  rightCounts = abs(rightRaw);
}

void printRampStatus() {
  if (millis() - previousPrintTime >= 500) {
    long leftCounts;
    long rightCounts;

    getEncoderCounts(leftCounts, rightCounts);

    Serial.print("Ramp climb");

    if (rampComplete) {
      Serial.print(" COMPLETE");
    } else if (rampFailed) {
      Serial.print(" FAILED");
    } else {
      Serial.print(" RUNNING");
    }

    Serial.print(" | Left counts: ");
    Serial.print(leftCounts);

    Serial.print(" | Right counts: ");
    Serial.print(rightCounts);

    Serial.print(" | Left motor: ");
    Serial.print(currentLeftMotorSpeed);

    Serial.print(" | Right motor: ");
    Serial.print(currentRightMotorSpeed);

    Serial.print(" | Time: ");
    Serial.print((millis() - rampStartTime) / 1000);

    Serial.println(" sec");

    previousPrintTime = millis();
  }
}

void updateLeftEncoder() {
  if (digitalRead(leftEncoderA) == digitalRead(leftEncoderB)) {
    leftEncoderCount--;
  } else {
    leftEncoderCount++;
  }
}

void updateRightEncoder() {
  if (digitalRead(rightEncoderA) == digitalRead(rightEncoderB)) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}
