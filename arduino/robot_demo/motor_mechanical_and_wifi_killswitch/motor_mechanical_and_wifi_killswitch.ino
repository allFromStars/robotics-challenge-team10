#include <Wire.h>
#include <Motoron.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// =====================
// WIFI SETTINGS
// =====================
const char* ssid = "Xia";
const char* password = "He600910";

WiFiUDP udp;
const int UDP_PORT = 4210;
char packetBuffer[255];

// =====================
// MOTOR SETUP
// =====================
MotoronI2C mc(16);

const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;

const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;

// =====================
// ENCODERS
// =====================
const int leftEncoderPinA = D2;
const int leftEncoderPinB = D3;

const int rightEncoderPinA = D4;
const int rightEncoderPinB = D5;

volatile long leftEncoderPos = 0;
volatile long rightEncoderPos = 0;

// =====================
// BUTTON AND LED
// =====================
const int buttonPin = D8;
const int redLedPin = D9;

// =====================
// MOVEMENT
// =====================
const int FORWARD_SPEED = -300;

int currentMode = 0;

const int MODE_STOP = 0;
const int MODE_FORWARD = 1;
const int MODE_BACKWARD = 2;
const int MODE_TURN_LEFT = 3;
const int MODE_TURN_RIGHT = 4;

// =====================
// BUTTON DEBOUNCE
// =====================
bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// =====================
// LED BLINK
// =====================
unsigned long lastBlinkTime = 0;
bool ledState = LOW;

const unsigned long blinkInterval = 500;

// =====================
// PRINTING
// =====================
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("SYSTEM STARTING - MOTOR + MECHANICAL KILL + WIFI KILL");

  // Motoron setup
  Wire1.begin();
  mc.setBus(&Wire1);

  mc.reinitialize();
  mc.clearResetFlag();

  mc.setCommandTimeoutMilliseconds(1000);

  mc.setMaxAcceleration(LEFT_MOTOR_CHANNEL, 200);
  mc.setMaxDeceleration(LEFT_MOTOR_CHANNEL, 300);

  mc.setMaxAcceleration(RIGHT_MOTOR_CHANNEL, 200);
  mc.setMaxDeceleration(RIGHT_MOTOR_CHANNEL, 300);

  // Encoder setup
  pinMode(leftEncoderPinA, INPUT_PULLUP);
  pinMode(leftEncoderPinB, INPUT_PULLUP);

  pinMode(rightEncoderPinA, INPUT_PULLUP);
  pinMode(rightEncoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), readLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), readRightEncoder, CHANGE);

  // Button and LED setup
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(redLedPin, OUTPUT);

  stopMotors();

  // WiFi setup
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");

  Serial.print("Arduino IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);

  Serial.print("Listening on UDP port ");
  Serial.println(UDP_PORT);

  Serial.println("Current mode: STOP");
  Serial.println("Button cycles modes. UDP STOP forces STOP.");
}

void loop() {
  mc.resetCommandTimeout();

  checkButton();
  checkUDPStop();

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

// =====================
// WIFI KILL SWITCH
// =====================
void checkUDPStop() {
  int packetSize = udp.parsePacket();

  if (packetSize) {
    int len = udp.read(packetBuffer, 255);

    if (len > 0) {
      packetBuffer[len] = '\0';
    }

    Serial.print("UDP received: ");
    Serial.println(packetBuffer);

    if (strcmp(packetBuffer, "STOP") == 0) {
      currentMode = MODE_STOP;
      stopMotors();

      Serial.println("WIFI KILL SWITCH: ROBOT STOPPED");
    }
  }
}

// =====================
// MECHANICAL BUTTON
// =====================
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

// =====================
// MODES
// =====================
void runCurrentMode() {
  if (currentMode == MODE_STOP) {
    stopMotors();

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

// =====================
// MOTORS
// =====================
void driveMotors(int leftSpeed, int rightSpeed) {
  mc.setSpeed(LEFT_MOTOR_CHANNEL, LEFT_DIR * leftSpeed);
  mc.setSpeed(RIGHT_MOTOR_CHANNEL, RIGHT_DIR * rightSpeed);
}

void stopMotors() {
  mc.setSpeed(LEFT_MOTOR_CHANNEL, 0);
  mc.setSpeed(RIGHT_MOTOR_CHANNEL, 0);
}

// =====================
// LED
// =====================
void blinkStoppedLed() {
  if (millis() - lastBlinkTime >= blinkInterval) {
    lastBlinkTime = millis();
    ledState = !ledState;
    digitalWrite(redLedPin, ledState);
  }
}

// =====================
// PRINT MODE
// =====================
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

// =====================
// ENCODERS
// =====================
void readLeftEncoder() {
  if (digitalRead(leftEncoderPinA) == digitalRead(leftEncoderPinB)) {
    leftEncoderPos--;
  } else {
    leftEncoderPos++;
  }
}

void readRightEncoder() {
  if (digitalRead(rightEncoderPinA) == digitalRead(rightEncoderPinB)) {
    rightEncoderPos--;
  } else {
    rightEncoderPos++;
  }
}