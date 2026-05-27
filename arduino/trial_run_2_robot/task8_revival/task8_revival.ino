#include <Wire.h>
#include <Motoron.h>

MotoronI2C motorController(16);

// Motor channels on the Motoron board
const int leftMotor = 1;
const int rightMotor = 2;

// Used to correct motor direction based on how the motors are mounted
const int leftMotorDir = 1;
const int rightMotorDir = -1;

// LED pins
const int redLed = 52;
const int greenLed = 48;

// Bumper switch pin
const int bumperSwitch = 8;

// Distance sensors use UART
const unsigned long sensorBaud = 921600;

// Each distance sensor sends a 16-byte data frame
uint8_t frontSensorData[16];
uint8_t sensor2Data[16];
uint8_t sensor3Data[16];
uint8_t sensor4Data[16];

// Distance readings in millimetres
uint32_t frontDistanceMm = 0;
uint32_t sensor2DistanceMm = 0;
uint32_t sensor3DistanceMm = 0;
uint32_t sensor4DistanceMm = 0;

// The robot has 60 seconds to make contact
const unsigned long rescueTimeLimit = 60000;

// P controller values
const uint32_t targetDistanceMm = 140;   // distance where the robot should be moving slowly
const int minDriveSpeed = 120;           // slowest speed that still moves the robot
const int maxDriveSpeed = 350;           // fastest allowed approach speed
const float distanceGain = 0.45;         // Kp value for the P controller

int currentDriveSpeed = 0;

// Revival states
const int APPROACHING_TARGET = 0;
const int REVIVAL_DONE = 1;
const int RESCUE_FAILED = 2;

int revivalState = APPROACHING_TARGET;

unsigned long rescueStartTime = 0;
unsigned long lastPrintTime = 0;

bool timeoutMessageShown = false;

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(10);
  }

  Serial.println();
  Serial.println("Robot revival test starting...");
  Serial.println("Red LED = revival still in progress.");
  Serial.println("Green LED = bumper contact made, revival successful.");
  Serial.println("Front distance sensor is used to control approach speed.");
  Serial.println("Approach speed is controlled using a P controller.");
  Serial.println();

  Wire1.begin();

  delay(500);

  motorController.setBus(&Wire1);
  motorController.reinitialize();
  motorController.clearResetFlag();

  motorController.setCommandTimeoutMilliseconds(1000);

  motorController.setMaxAcceleration(leftMotor, 120);
  motorController.setMaxDeceleration(leftMotor, 250);

  motorController.setMaxAcceleration(rightMotor, 120);
  motorController.setMaxDeceleration(rightMotor, 250);

  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);

  pinMode(bumperSwitch, INPUT_PULLUP);

  Serial1.begin(sensorBaud);
  Serial2.begin(sensorBaud);
  Serial3.begin(sensorBaud);
  Serial4.begin(sensorBaud);

  delay(1000);

  clearSensorBuffers();

  stopRobot();
  turnRedLedOn();

  rescueStartTime = millis();
}

void loop() {
  motorController.resetCommandTimeout();

  runRevivalTask();

  printStatus();
}

// Main revival behaviour
void runRevivalTask() {
  readDistanceSensors();

  bool bumperPressed = digitalRead(bumperSwitch) == LOW;
  unsigned long timeUsed = millis() - rescueStartTime;

  // Stop the attempt if the robot has not made contact within 60 seconds
  if (revivalState == APPROACHING_TARGET && timeUsed >= rescueTimeLimit) {
    revivalState = RESCUE_FAILED;
  }

  if (revivalState == APPROACHING_TARGET) {
    turnRedLedOn();

    if (bumperPressed) {
      stopRobot();
      turnGreenLedOn();

      revivalState = REVIVAL_DONE;

      Serial.println();
      Serial.println("REVIVAL SUCCESSFUL");
      Serial.println("Bumper contact made. Robot stopped and green LED turned on.");
      Serial.println();
    } else {
      currentDriveSpeed = calculateApproachSpeed(frontDistanceMm);
      driveRobot(currentDriveSpeed, currentDriveSpeed);
    }
  }

  else if (revivalState == REVIVAL_DONE) {
    stopRobot();
    turnGreenLedOn();
  }

  else if (revivalState == RESCUE_FAILED) {
    stopRobot();
    turnRedLedOn();

    if (!timeoutMessageShown) {
      Serial.println();
      Serial.println("The robot did not make contact within 60 seconds.");
      Serial.println("Revival attempt stopped.");
      Serial.println();

      timeoutMessageShown = true;
    }
  }
}

// P controller for approach speed
int calculateApproachSpeed(uint32_t distanceMm) {
  // If the sensor gives no reading, move slowly rather than stopping suddenly
  if (distanceMm == 0) {
    return -minDriveSpeed;
  }

  long distanceError = distanceMm - targetDistanceMm;

  // Once the robot is at or inside the target distance, keep the error at zero
  if (distanceError < 0) {
    distanceError = 0;
  }

  int speed = minDriveSpeed + (distanceGain * distanceError);

  // Keep the speed inside safe limits
  if (speed > maxDriveSpeed) {
    speed = maxDriveSpeed;
  }

  if (speed < minDriveSpeed) {
    speed = minDriveSpeed;
  }

  // On our robot, negative speed means forward
  return -speed;
}

// Read all connected distance sensors
void readDistanceSensors() {
  if (readSensorFrame(Serial1, frontSensorData)) {
    frontDistanceMm = getDistanceFromFrame(frontSensorData);
  }

  if (readSensorFrame(Serial2, sensor2Data)) {
    sensor2DistanceMm = getDistanceFromFrame(sensor2Data);
  }

  if (readSensorFrame(Serial3, sensor3Data)) {
    sensor3DistanceMm = getDistanceFromFrame(sensor3Data);
  }

  if (readSensorFrame(Serial4, sensor4Data)) {
    sensor4DistanceMm = getDistanceFromFrame(sensor4Data);
  }
}

// Reads one 16-byte distance sensor frame
bool readSensorFrame(HardwareSerial &sensorPort, uint8_t *frame) {
  while (sensorPort.available()) {
    uint8_t firstByte = sensorPort.read();

    // Valid frames start with 0x57
    if (firstByte != 0x57) {
      continue;
    }

    frame[0] = firstByte;

    unsigned long startTime = millis();
    int index = 1;

    while (index < 16 && millis() - startTime < 50) {
      if (sensorPort.available()) {
        frame[index] = sensorPort.read();
        index++;
      }
    }

    if (index < 16) {
      return false;
    }

    // Check the frame header
    if (frame[1] != 0x00 || frame[2] != 0xFF) {
      return false;
    }

    uint8_t checksum = 0;

    for (int i = 0; i < 15; i++) {
      checksum += frame[i];
    }

    // Ignore the frame if the checksum is wrong
    if (checksum != frame[15]) {
      return false;
    }

    return true;
  }

  return false;
}

// Distance value is stored across bytes 8, 9 and 10
uint32_t getDistanceFromFrame(uint8_t *frame) {
  return ((uint32_t)frame[8]) |
         ((uint32_t)frame[9] << 8) |
         ((uint32_t)frame[10] << 16);
}

// Clear any old sensor data before starting
void clearSensorBuffers() {
  while (Serial1.available()) {
    Serial1.read();
  }

  while (Serial2.available()) {
    Serial2.read();
  }

  while (Serial3.available()) {
    Serial3.read();
  }

  while (Serial4.available()) {
    Serial4.read();
  }
}

// Motor control
void driveRobot(int leftSpeed, int rightSpeed) {
  motorController.setSpeed(leftMotor, leftMotorDir * leftSpeed);
  motorController.setSpeed(rightMotor, rightMotorDir * rightSpeed);
}

void stopRobot() {
  motorController.setSpeed(leftMotor, 0);
  motorController.setSpeed(rightMotor, 0);
  currentDriveSpeed = 0;
}

// LED control
void turnRedLedOn() {
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, LOW);
}

void turnGreenLedOn() {
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, HIGH);
}

// Print useful debugging information once per second
void printStatus() {
  if (millis() - lastPrintTime >= 1000) {
    Serial.print("Robot status: ");

    if (revivalState == APPROACHING_TARGET) {
      Serial.print("approaching target");
    } else if (revivalState == REVIVAL_DONE) {
      Serial.print("revival complete");
    } else if (revivalState == RESCUE_FAILED) {
      Serial.print("stopped - time ran out");
    }

    Serial.print(" | Front distance: ");
    Serial.print(frontDistanceMm);
    Serial.print(" mm");

    Serial.print(" | Drive speed: ");
    Serial.print(currentDriveSpeed);

    Serial.print(" | Bumper: ");

    if (digitalRead(bumperSwitch) == LOW) {
      Serial.print("pressed");
    } else {
      Serial.print("not pressed");
    }

    Serial.print(" | Time used: ");
    Serial.print((millis() - rescueStartTime) / 1000);
    Serial.println(" sec");

    lastPrintTime = millis();
  }
}
