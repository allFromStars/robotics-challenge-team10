#include <Wire.h>
#include <Motoron.h>

// Initialize with address 16
MotoronI2C mc(16);

const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;

const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;
const int leftEncoderPinA = D2;
const int leftEncoderPinB = D3;

const int rightEncoderPinA = D4;
const int rightEncoderPinB = D5;

volatile long leftEncoderPos = 0;
volatile long rightEncoderPos = 0;

const int buttonPin = D8;
const int redLedPin = D9;
const int FORWARD_SPEED = -300;

int currentMode = 0;

const int MODE_STOP = 0;
const int MODE_FORWARD = 1;
const int MODE_BACKWARD = 2;
const int MODE_TURN_LEFT = 3;
const int MODE_TURN_RIGHT = 4;

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

unsigned long lastBlinkTime = 0;
bool ledState = LOW;

const unsigned long blinkInterval = 500;

unsigned long lastPrintTime = 0;


void setup() {

  Serial.begin(115200);
  Serial.println("SYSTEM STARTING - MODE TOGGLE PRACTICE");

  Wire1.begin();
  mc.setBus(&Wire1);

  mc.reinitialize();
  mc.clearResetFlag();

  mc.setCommandTimeoutMilliseconds(1000);

  mc.setMaxAcceleration(LEFT_MOTOR_CHANNEL, 200);
  mc.setMaxDeceleration(LEFT_MOTOR_CHANNEL, 300);

  mc.setMaxAcceleration(RIGHT_MOTOR_CHANNEL, 200);
  mc.setMaxDeceleration(RIGHT_MOTOR_CHANNEL, 300);

  pinMode(leftEncoderPinA, INPUT_PULLUP);
  pinMode(leftEncoderPinB, INPUT_PULLUP);

  pinMode(rightEncoderPinA, INPUT_PULLUP);
  pinMode(rightEncoderPinB, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(leftEncoderPinA),
    readLeftEncoder,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(rightEncoderPinA),
    readRightEncoder,
    CHANGE
  );

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(redLedPin, OUTPUT);

  stopMotors();

  Serial.println("Current mode: STOP");
  Serial.println("Press button to cycle modes.");
}


void loop() {

  mc.resetCommandTimeout();

  checkButton();

  runCurrentMode();

  if (millis() - lastPrintTime >= 500) {

    Serial.print("Mode: ");
    printModeName();

    Serial.print(" | Left Pos: ");
    Serial.print(leftEncoderPos);

    Serial.print(" | Right Pos: ");
    Serial.println(-rightEncoderPos);

    lastPrintTime = millis();
  }
}


void checkButton() {

  bool reading = digitalRead(buttonPin);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (reading != stableButtonState) {

      stableButtonState = reading;
      if (stableButtonState == LOW) {

        currentMode++;

        if (currentMode > MODE_TURN_RIGHT) {
          currentMode = MODE_STOP;
        }

        Serial.print("Button pressed. New mode: ");
        printModeName();
        Serial.println();
      }
    }
  }

  lastButtonReading = reading;
}


void runCurrentMode() {

  if (currentMode == MODE_STOP) {

  stopMotors();

  // Reset encoder positions
  leftEncoderPos = 0;
  rightEncoderPos = 0;

  blinkStoppedLed();
}

  else if (currentMode == MODE_FORWARD) {

    digitalWrite(redLedPin, LOW);
    driveMotors(FORWARD_SPEED, FORWARD_SPEED);
  }

  else if (currentMode == MODE_BACKWARD) {

    digitalWrite(redLedPin, LOW);
    driveMotors(-FORWARD_SPEED, -FORWARD_SPEED);
  }

  else if (currentMode == MODE_TURN_LEFT) {

    digitalWrite(redLedPin, LOW);
    driveMotors(-FORWARD_SPEED, FORWARD_SPEED);
  }

  else if (currentMode == MODE_TURN_RIGHT) {

    digitalWrite(redLedPin, LOW);
    driveMotors(FORWARD_SPEED, -FORWARD_SPEED);
  }
}


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


void blinkStoppedLed() {

  if (millis() - lastBlinkTime >= blinkInterval) {

    lastBlinkTime = millis();

    ledState = !ledState;

    digitalWrite(redLedPin, ledState);
  }
}


void printModeName() {

  if (currentMode == MODE_STOP) {
    Serial.print("STOP");
  }

  else if (currentMode == MODE_FORWARD) {
    Serial.print("FORWARD");
  }

  else if (currentMode == MODE_BACKWARD) {
    Serial.print("BACKWARD");
  }

  else if (currentMode == MODE_TURN_LEFT) {
    Serial.print("TURN LEFT");
  }

  else if (currentMode == MODE_TURN_RIGHT) {
    Serial.print("TURN RIGHT");
  }
}


void readLeftEncoder() {

  if (digitalRead(leftEncoderPinA) ==
      digitalRead(leftEncoderPinB)) {

    leftEncoderPos--;

  } else {

    leftEncoderPos++;
  }
}


void readRightEncoder() {

  if (digitalRead(rightEncoderPinA) ==
      digitalRead(rightEncoderPinB)) {

    rightEncoderPos--;

  } else {

    rightEncoderPos++;
  }
}
