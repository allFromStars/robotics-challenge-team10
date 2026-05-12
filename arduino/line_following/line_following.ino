#include <Wire.h>
#include <Motoron.h>

// =========================
// MOTOR SETUP
// =========================

MotoronI2C mc(16);

const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;  

const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;

const int BASE_SPEED = -300;

// =========================
// BUTTON + LED
// =========================

const int buttonPin = D8;
const int redLedPin = D9;

bool robotRunning = false;

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

unsigned long lastBlinkTime = 0;
bool ledState = LOW;

const unsigned long blinkInterval = 500;

// =========================
// QTR SENSOR ARRAY
// =========================

const uint8_t SensorCount = 9;

const uint8_t sensorPins[SensorCount] = {
  22, 23, 24, 25, 26, 27, 28, 29, 30
};

uint16_t sensorValues[SensorCount];

const uint16_t timeout = 2500;

// =========================
// SETUP
// =========================

void setup() {

  Serial.begin(115200);

  Wire1.begin();
  mc.setBus(&Wire1);

  mc.reinitialize();
  mc.clearResetFlag();

  mc.setCommandTimeoutMilliseconds(1000);

  mc.setMaxAcceleration(LEFT_MOTOR_CHANNEL, 200);
  mc.setMaxDeceleration(LEFT_MOTOR_CHANNEL, 300);

  mc.setMaxAcceleration(RIGHT_MOTOR_CHANNEL, 200);
  mc.setMaxDeceleration(RIGHT_MOTOR_CHANNEL, 300);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(redLedPin, OUTPUT);

  stopMotors();

  Serial.println("LINE FOLLOWING TEST");
}

// =========================
// MAIN LOOP
// =========================

void loop() {

  mc.resetCommandTimeout();

  checkButton();

  if (!robotRunning) {

    stopMotors();
    blinkStoppedLed();

    return;
  }

  digitalWrite(redLedPin, LOW);

  readQTR_RC();

  runBangBangLineFollowing();
}

// =========================
// LINE FOLLOWING
// =========================

void runBangBangLineFollowing() {

  long leftSum =
    sensorValues[0] +
    sensorValues[1] +
    sensorValues[2];

  long centerSum =
    sensorValues[3] +
    sensorValues[4] +
    sensorValues[5];

  long rightSum =
    sensorValues[6] +
    sensorValues[7] +
    sensorValues[8];

  Serial.print(leftSum);
  Serial.print(" | ");

  Serial.print(centerSum);
  Serial.print(" | ");

  Serial.println(rightSum);

  // IMPORTANT:
  // Assumes black line = larger values

  if (centerSum > leftSum &&
      centerSum > rightSum) {

    // Go forward
    driveMotors(BASE_SPEED, BASE_SPEED);
  }

  else if (leftSum > rightSum) {

    // Turn left
    driveMotors(BASE_SPEED / 2, BASE_SPEED);
  }

  else if (rightSum > leftSum) {

    // Turn right
    driveMotors(BASE_SPEED, BASE_SPEED / 2);
  }

  else {

    stopMotors();
  }
}

// =========================
// MOTOR FUNCTIONS
// =========================

void driveMotors(int leftSpeed, int rightSpeed) {

  mc.setSpeed(
    LEFT_MOTOR_CHANNEL,
    LEFT_DIR * leftSpeed
  );

  mc.setSpeed(
    RIGHT_MOTOR_CHANNEL,
    RIGHT_DIR * rightSpeed
  );
}

void stopMotors() {

  mc.setSpeed(LEFT_MOTOR_CHANNEL, 0);
  mc.setSpeed(RIGHT_MOTOR_CHANNEL, 0);
}

// =========================
// BUTTON
// =========================

void checkButton() {

  bool reading = digitalRead(buttonPin);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (reading != stableButtonState) {

      stableButtonState = reading;

      if (stableButtonState == LOW) {

        robotRunning = !robotRunning;

        Serial.print("Robot running: ");
        Serial.println(robotRunning);
      }
    }
  }

  lastButtonReading = reading;
}

// =========================
// LED BLINK
// =========================

void blinkStoppedLed() {

  if (millis() - lastBlinkTime >= blinkInterval) {

    lastBlinkTime = millis();

    ledState = !ledState;

    digitalWrite(redLedPin, ledState);
  }
}

// =========================
// QTR SENSOR READING
// =========================

void readQTR_RC() {

  for (uint8_t i = 0; i < SensorCount; i++) {

    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }

  delayMicroseconds(10);

  for (uint8_t i = 0; i < SensorCount; i++) {

    pinMode(sensorPins[i], INPUT);

    sensorValues[i] = timeout;
  }

  unsigned long startTime = micros();

  bool allDone = false;

  while (!allDone &&
         (micros() - startTime < timeout)) {

    allDone = true;

    for (uint8_t i = 0; i < SensorCount; i++) {

      if (sensorValues[i] == timeout) {

        if (digitalRead(sensorPins[i]) == LOW) {

          sensorValues[i] =
            micros() - startTime;

        } else {

          allDone = false;
        }
      }
    }
  }
}