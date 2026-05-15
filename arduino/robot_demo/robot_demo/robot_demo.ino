#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>

MotoronI2C mc(16);


// RFID
#define RFID_ADDR 0x28
MFRC522_I2C rfid(RFID_ADDR, -1, &Wire1);

// MOTORS
const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;

const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;

const int FORWARD_SPEED = -300;

// Speeds for test 5
const int FORWARD_SLOW = -150;
const int FORWARD_MEDIUM = -300;
const int FORWARD_FAST = -500;
const int FORWARD_FASTEST = -800;

// ENCODER BASED U-TURN
const int UTURN_SPEED = -400;
const int UTURN_SLOW_SPEED = -300;

const long UTURN_180_COUNTS = 1200 * 4.6;
const long UTURN_SLOWDOWN_COUNTS = 250;

bool uTurnStarted = false;
bool uTurnFinished = false;

// 90 DEGREE TURN TEST
const int TURN90_SPEED = -350;
const int TURN90_SLOW_SPEED = -350;

const long LEFT_90_COUNTS = 2500;
const long RIGHT_90_COUNTS = 2500;

const long TURN90_SLOWDOWN_COUNTS = 250;

bool turn90InProgress = false;

// -1 = left, 1 = right
int activeTurn90Direction = 0;
int nextTurn90Direction = -1;


// ENCODERS
const int leftEncoderPinA = D2;
const int leftEncoderPinB = D3;

const int rightEncoderPinA = D4;
const int rightEncoderPinB = D5;

volatile long leftEncoderPos = 0;
volatile long rightEncoderPos = 0;

// BUTTON AND RGB LED
const int buttonPin = D8;

// Common cathode RGB LED
const int r_led = D52;
const int g_led = D50;

// TEST NUMBER
int currentTest = 0;

// MOTOR MODES FOR TEST 1
int currentMode = 0;

const int MODE_STOP = 0;
const int MODE_FORWARD = 1;
const int MODE_BACKWARD = 2;
const int MODE_TURN_LEFT = 3;
const int MODE_TURN_RIGHT = 4;

// SPEED MODES FOR TEST 5
int currentSpeedMode = 0;

const int SPEED_STOP = 0;
const int SPEED_SLOW = 1;
const int SPEED_MEDIUM = 2;
const int SPEED_FAST = 3;
const int SPEED_FASTEST = 4;

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// RGB LED BLINk
unsigned long lastBlinkTime = 0;
bool ledState = LOW;
const unsigned long blinkInterval = 500;

unsigned long lastPrintTime = 0;

bool rfidMessagePrinted = false;
bool tofMessagePrinted = false;
bool irMessagePrinted = false;

// DISTANCE SENSORS
const unsigned long SENSOR_BAUD = 921600;

uint8_t frame1[16];
uint8_t frame2[16];
uint8_t frame3[16];
uint8_t frame4[16];

uint32_t distance1 = 0;
uint32_t distance2 = 0;
uint32_t distance3 = 0;
uint32_t distance4 = 0;

unsigned long lastTOFPrintTime = 0;

// =====================
// IR / QTR SENSORS
// =====================
const uint8_t SensorCount = 9;

const uint8_t sensorPins[SensorCount] = {
  22, 23, 24, 25, 26, 27, 28, 29, 30
};

uint16_t sensorValues[SensorCount];

const uint16_t timeout = 2500; // microseconds

unsigned long lastIRPrintTime = 0;

// =====================
// SETUP
// =====================
void setup() {

  Serial.begin(115200);

  while (!Serial) {
    delay(10);
  }

  delay(1000);

  Serial.println();
  Serial.println("Robot test code started");
  Serial.println("Press 1 for kill switch test");
  Serial.println("Press 2 for RFID test");
  Serial.println("Press 3 for distance sensor test");
  Serial.println("Press 4 for IR sensor test");
  Serial.println("Press 5 for forward speed test");
  Serial.println("Press 6 for encoder U-turn test");
  Serial.println("Press 7 for left/right 90 degree turn test");
  Serial.println();

  Wire1.begin();

  delay(500);

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

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), readLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), readRightEncoder, CHANGE);

  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(r_led, OUTPUT);
  pinMode(g_led, OUTPUT);

  rfid.PCD_Init();
  delay(300);

  Serial1.begin(SENSOR_BAUD);
  Serial2.begin(SENSOR_BAUD);
  Serial3.begin(SENSOR_BAUD);
  Serial4.begin(SENSOR_BAUD);

  delay(500);

  stopMotors();
  rgbOff();
}

// =====================
// MAIN LOOP
// =====================
void loop() {

  checkSerial();

  if (currentTest == 1) {
    runKillSwitchTest();
  }

  if (currentTest == 2) {
    runRFIDTest();
  }

  if (currentTest == 3) {
    runTOFTest();
  }

  if (currentTest == 4) {
    runIRTest();
  }

  if (currentTest == 5) {
    runForwardSpeedTest();
  }

  if (currentTest == 6) {
    runUTurnTest();
  }

  if (currentTest == 7) {
    runTurn90ToggleTest();
  }
}

// =====================
// SERIAL TEST SELECTION
// =====================
void checkSerial() {

  if (Serial.available() > 0) {

    char input = Serial.read();

    if (input == '1') {

      currentTest = 1;

      currentMode = MODE_STOP;
      currentSpeedMode = SPEED_STOP;

      resetEncoders();

      rfidMessagePrinted = false;
      tofMessagePrinted = false;
      irMessagePrinted = false;

      uTurnStarted = false;
      uTurnFinished = false;

      turn90InProgress = false;
      activeTurn90Direction = 0;

      resetButtonDebounce();

      stopMotors();
      rgbOff();

      Serial.println();
      Serial.println("Kill switch test started");
      Serial.println("Press the button to change motor mode");
      Serial.println();
    }

    if (input == '2') {

      currentTest = 2;

      currentMode = MODE_STOP;
      currentSpeedMode = SPEED_STOP;

      stopMotors();
      rgbOff();

      rfidMessagePrinted = false;
      tofMessagePrinted = false;
      irMessagePrinted = false;

      uTurnStarted = false;
      uTurnFinished = false;

      turn90InProgress = false;
      activeTurn90Direction = 0;

      Wire1.begin();
      delay(100);

      rfid.PCD_Init();
      delay(300);

      Serial.println();
      Serial.println("RFID test started");
      Serial.println("Put the card near the reader");
      Serial.println();
    }

    if (input == '3') {

      currentTest = 3;

      currentMode = MODE_STOP;
      currentSpeedMode = SPEED_STOP;

      stopMotors();
      rgbOff();

      rfidMessagePrinted = false;
      tofMessagePrinted = false;
      irMessagePrinted = false;

      uTurnStarted = false;
      uTurnFinished = false;

      turn90InProgress = false;
      activeTurn90Direction = 0;

      distance1 = 0;
      distance2 = 0;
      distance3 = 0;
      distance4 = 0;

      Serial1.begin(SENSOR_BAUD);
      Serial2.begin(SENSOR_BAUD);
      Serial3.begin(SENSOR_BAUD);
      Serial4.begin(SENSOR_BAUD);

      clearTOFSerialBuffers();

      Serial.println();
      Serial.println("Distance sensor test started");
      Serial.println("Reading 4 TOF sensors");
      Serial.println();
    }

    if (input == '4') {

      currentTest = 4;

      currentMode = MODE_STOP;
      currentSpeedMode = SPEED_STOP;

      stopMotors();
      rgbOff();

      rfidMessagePrinted = false;
      tofMessagePrinted = false;
      irMessagePrinted = false;

      uTurnStarted = false;
      uTurnFinished = false;

      turn90InProgress = false;
      activeTurn90Direction = 0;

      Serial.println();
      Serial.println("IR sensor test started");
      Serial.println("QTR-HD-09RC manual RC readings");
      Serial.println("Higher value = darker / weaker reflection");
      Serial.println("Lower value  = lighter / stronger reflection");
      Serial.println();
      Serial.println("S1\tS2\tS3\tS4\tS5\tS6\tS7\tS8\tS9");
      Serial.println();
    }

    if (input == '5') {

      currentTest = 5;

      currentMode = MODE_STOP;
      currentSpeedMode = SPEED_STOP;

      resetEncoders();

      rfidMessagePrinted = false;
      tofMessagePrinted = false;
      irMessagePrinted = false;

      uTurnStarted = false;
      uTurnFinished = false;

      turn90InProgress = false;
      activeTurn90Direction = 0;

      resetButtonDebounce();

      stopMotors();
      rgbOff();

      Serial.println();
      Serial.println("Forward speed test started");
      Serial.println("Button toggles:");
      Serial.println("STOP -> SLOW -> MEDIUM -> FAST -> FASTEST -> STOP");
      Serial.println();
    }

    if (input == '6') {

      currentTest = 6;

      currentMode = MODE_STOP;
      currentSpeedMode = SPEED_STOP;

      resetEncoders();

      rfidMessagePrinted = false;
      tofMessagePrinted = false;
      irMessagePrinted = false;

      uTurnStarted = false;
      uTurnFinished = false;

      turn90InProgress = false;
      activeTurn90Direction = 0;

      stopMotors();
      rgbOff();

      Serial.println();
      Serial.println("Encoder U-turn test started");
      Serial.print("Target counts for 180 degrees: ");
      Serial.println(UTURN_180_COUNTS);
      Serial.println();
    }

    if (input == '7') {

      currentTest = 7;

      currentMode = MODE_STOP;
      currentSpeedMode = SPEED_STOP;

      resetEncoders();

      rfidMessagePrinted = false;
      tofMessagePrinted = false;
      irMessagePrinted = false;

      uTurnStarted = false;
      uTurnFinished = false;

      turn90InProgress = false;
      activeTurn90Direction = 0;

      nextTurn90Direction = -1;

      resetButtonDebounce();

      stopMotors();
      rgbOff();

      Serial.println();
      Serial.println("90 degree turn test started");
      Serial.println("Press button to alternate turns:");
      Serial.println("1st button press = LEFT 90");
      Serial.println("2nd button press = RIGHT 90");
      Serial.println("3rd button press = LEFT 90 again");
      Serial.print("LEFT target counts: ");
      Serial.println(LEFT_90_COUNTS);
      Serial.print("RIGHT target counts: ");
      Serial.println(RIGHT_90_COUNTS);
      Serial.println();
    }
  }
}

// =====================
// TEST 1: KILL SWITCH TEST
// =====================
void runKillSwitchTest() {

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

        Serial.print("Button pressed. Mode is now: ");
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

    resetEncoders();

    blinkRedRGB();
  }

  else if (currentMode == MODE_FORWARD) {

    rgbOff();

    driveMotors(FORWARD_SPEED, FORWARD_SPEED);
  }

  else if (currentMode == MODE_BACKWARD) {

    rgbOff();

    driveMotors(-FORWARD_SPEED, -FORWARD_SPEED);
  }

  else if (currentMode == MODE_TURN_LEFT) {

    rgbOff();

    driveMotors(-FORWARD_SPEED, FORWARD_SPEED);
  }

  else if (currentMode == MODE_TURN_RIGHT) {

    rgbOff();

    driveMotors(FORWARD_SPEED, -FORWARD_SPEED);
  }
}

// =====================
// TEST 5: FORWARD SPEED TEST
// =====================
void runForwardSpeedTest() {

  mc.resetCommandTimeout();

  checkSpeedButton();

  runCurrentSpeedMode();

  if (millis() - lastPrintTime >= 500) {

    Serial.print("Speed mode: ");
    printSpeedModeName();

    Serial.print(" | Left Pos: ");
    Serial.print(leftEncoderPos);

    Serial.print(" | Right Pos: ");
    Serial.println(-rightEncoderPos);

    lastPrintTime = millis();
  }
}

void checkSpeedButton() {

  bool reading = digitalRead(buttonPin);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (reading != stableButtonState) {

      stableButtonState = reading;

      if (stableButtonState == LOW) {

        currentSpeedMode++;

        if (currentSpeedMode > SPEED_FASTEST) {
          currentSpeedMode = SPEED_STOP;
        }

        Serial.print("Button pressed. Speed mode is now: ");
        printSpeedModeName();
        Serial.println();
      }
    }
  }

  lastButtonReading = reading;
}

void runCurrentSpeedMode() {

  if (currentSpeedMode == SPEED_STOP) {

    stopMotors();

    resetEncoders();

    blinkRedRGB();
  }

  else if (currentSpeedMode == SPEED_SLOW) {

    rgbOff();

    driveMotors(FORWARD_SLOW, FORWARD_SLOW);
  }

  else if (currentSpeedMode == SPEED_MEDIUM) {

    rgbOff();

    driveMotors(FORWARD_MEDIUM, FORWARD_MEDIUM);
  }

  else if (currentSpeedMode == SPEED_FAST) {

    rgbOff();

    driveMotors(FORWARD_FAST, FORWARD_FAST);
  }

  else if (currentSpeedMode == SPEED_FASTEST) {

    rgbOff();

    driveMotors(FORWARD_FASTEST, FORWARD_FASTEST);
  }
}

void printSpeedModeName() {

  if (currentSpeedMode == SPEED_STOP) {
    Serial.print("STOP");
  }

  else if (currentSpeedMode == SPEED_SLOW) {
    Serial.print("FORWARD SLOW");
  }

  else if (currentSpeedMode == SPEED_MEDIUM) {
    Serial.print("FORWARD MEDIUM");
  }

  else if (currentSpeedMode == SPEED_FAST) {
    Serial.print("FORWARD FAST");
  }

  else if (currentSpeedMode == SPEED_FASTEST) {
    Serial.print("FORWARD FASTEST");
  }
}

// =====================
// TEST 6: ENCODER U-TURN
// =====================
void runUTurnTest() {

  mc.resetCommandTimeout();

  if (!uTurnStarted && !uTurnFinished) {

    resetEncoders();

    uTurnStarted = true;

    rgbOff();

    Serial.println("Encoder U-turn moving...");
  }

  if (uTurnStarted && !uTurnFinished) {

    long turnCounts = getAverageTurnCounts();
    long remainingCounts = UTURN_180_COUNTS - turnCounts;

    if (turnCounts >= UTURN_180_COUNTS) {

      stopMotors();

      uTurnStarted = false;
      uTurnFinished = true;

      Serial.println("Encoder U-turn finished. Robot stopped.");
      Serial.println();
    }

    else {

      if (remainingCounts < UTURN_SLOWDOWN_COUNTS) {
        driveMotors(UTURN_SLOW_SPEED, -UTURN_SLOW_SPEED);
      } else {
        driveMotors(UTURN_SPEED, -UTURN_SPEED);
      }
    }

    if (millis() - lastPrintTime >= 300) {

      Serial.print("U-turn counts: ");
      Serial.print(turnCounts);

      Serial.print(" / ");
      Serial.print(UTURN_180_COUNTS);

      Serial.print(" | Left Pos: ");
      Serial.print(leftEncoderPos);

      Serial.print(" | Right Pos: ");
      Serial.println(-rightEncoderPos);

      lastPrintTime = millis();
    }
  }

  if (uTurnFinished) {
    stopMotors();
    blinkRedRGB();
  }
}

// =====================
// TEST 7: LEFT/RIGHT 90 DEGREE TURNS
// =====================
void runTurn90ToggleTest() {

  mc.resetCommandTimeout();

  checkTurn90Button();

  if (!turn90InProgress) {
    stopMotors();
    blinkRedRGB();
  }

  if (turn90InProgress) {

    long targetCounts;

    if (activeTurn90Direction == -1) {
      targetCounts = LEFT_90_COUNTS;
    } else {
      targetCounts = RIGHT_90_COUNTS;
    }

    long turnCounts = getAverageTurnCounts();
    long remainingCounts = targetCounts - turnCounts;

    if (turnCounts >= targetCounts) {

      stopMotors();

      turn90InProgress = false;
      activeTurn90Direction = 0;

      Serial.println("90 degree turn finished. Robot stopped.");

      if (nextTurn90Direction == -1) {
        Serial.println("Next button press = LEFT 90");
      } else {
        Serial.println("Next button press = RIGHT 90");
      }

      Serial.println();
    }

    else {

      int speedToUse;

      if (remainingCounts < TURN90_SLOWDOWN_COUNTS) {
        speedToUse = TURN90_SLOW_SPEED;
      } else {
        speedToUse = TURN90_SPEED;
      }

      if (activeTurn90Direction == 1) {
        driveMotors(speedToUse, -speedToUse);
      }

      else if (activeTurn90Direction == -1) {
        driveMotors(-speedToUse, speedToUse);
      }
    }

    if (millis() - lastPrintTime >= 300) {

      Serial.print("90 turn counts: ");
      Serial.print(turnCounts);

      Serial.print(" / ");
      Serial.print(targetCounts);

      Serial.print(" | Left Pos: ");
      Serial.print(leftEncoderPos);

      Serial.print(" | Right Pos: ");
      Serial.println(-rightEncoderPos);

      lastPrintTime = millis();
    }
  }
}

void checkTurn90Button() {

  bool reading = digitalRead(buttonPin);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (reading != stableButtonState) {

      stableButtonState = reading;

      if (stableButtonState == LOW) {

        if (!turn90InProgress) {

          resetEncoders();

          activeTurn90Direction = nextTurn90Direction;
          turn90InProgress = true;

          if (activeTurn90Direction == -1) {

            Serial.println("Button pressed. LEFT 90 started.");

            nextTurn90Direction = 1;
          }

          else if (activeTurn90Direction == 1) {

            Serial.println("Button pressed. RIGHT 90 started.");

            nextTurn90Direction = -1;
          }
        }
      }
    }
  }

  lastButtonReading = reading;
}

long getAverageTurnCounts() {

  noInterrupts();

  long leftCounts = leftEncoderPos;
  long rightCounts = rightEncoderPos;

  interrupts();

  leftCounts = abs(leftCounts);
  rightCounts = abs(rightCounts);

  return (leftCounts + rightCounts) / 2;
}

void resetEncoders() {

  noInterrupts();

  leftEncoderPos = 0;
  rightEncoderPos = 0;

  interrupts();
}

// =====================
// MOTOR FUNCTIONS
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
// RGB LED FUNCTIONS
// =====================
void rgbOff() {
  digitalWrite(r_led, LOW);
  digitalWrite(g_led, LOW);
}

void setRed() {
  digitalWrite(r_led, HIGH);
  digitalWrite(g_led, LOW);
}

void setGreen() {
  digitalWrite(r_led, LOW);
  digitalWrite(g_led, HIGH);
}

void blinkRedRGB() {

  if (millis() - lastBlinkTime >= blinkInterval) {

    lastBlinkTime = millis();

    ledState = !ledState;

    if (ledState) {
      setRed();
    } else {
      rgbOff();
    }
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

void resetButtonDebounce() {

  lastButtonReading = digitalRead(buttonPin);
  stableButtonState = lastButtonReading;
  lastDebounceTime = millis();
}

// =====================
// TEST 2: RFID TEST
// =====================
void runRFIDTest() {

  static unsigned long lastScanPrint = 0;

  if (!rfidMessagePrinted) {

    Serial.println("RFID ready");
    Serial.println("Scanning for RFID card...");
    Serial.println();

    rfidMessagePrinted = true;
  }

  if (millis() - lastScanPrint >= 1000) {
    Serial.println("Scanning...");
    lastScanPrint = millis();
  }

  if (!rfid.PICC_IsNewCardPresent()) {
    return;
  }

  if (!rfid.PICC_ReadCardSerial()) {
    return;
  }

  Serial.print("UID: ");

  for (byte i = 0; i < rfid.uid.size; i++) {

    if (rfid.uid.uidByte[i] < 0x10) {
      Serial.print("0");
    }

    Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.print(" ");
  }

  Serial.println();
  Serial.println("Card read");
  Serial.println();

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  delay(1000);
}

// =====================
// TEST 3: DISTANCE SENSOR TEST
// =====================
void runTOFTest() {

  if (!tofMessagePrinted) {
    Serial.println("TOF sensors ready");
    Serial.println("Scanning distances...");
    Serial.println();
    tofMessagePrinted = true;
  }

  if (readTOFFrame(Serial1, frame1)) {
    distance1 = getDistanceMM(frame1);
  }

  if (readTOFFrame(Serial2, frame2)) {
    distance2 = getDistanceMM(frame2);
  }

  if (readTOFFrame(Serial3, frame3)) {
    distance3 = getDistanceMM(frame3);
  }

  if (readTOFFrame(Serial4, frame4)) {
    distance4 = getDistanceMM(frame4);
  }

  if (millis() - lastTOFPrintTime >= 200) {

    Serial.print("Sensor 1: ");
    Serial.print(distance1);
    Serial.print(" mm");

    Serial.print(" | Sensor 2: ");
    Serial.print(distance2);
    Serial.print(" mm");

    Serial.print(" | Sensor 3: ");
    Serial.print(distance3);
    Serial.print(" mm");

    Serial.print(" | Sensor 4: ");
    Serial.print(distance4);
    Serial.println(" mm");

    lastTOFPrintTime = millis();
  }
}

bool readTOFFrame(HardwareSerial &port, uint8_t *frame) {

  while (port.available()) {

    uint8_t b = port.read();

    if (b != 0x57) {
      continue;
    }

    frame[0] = b;

    unsigned long startTime = millis();
    int index = 1;

    while (index < 16 && millis() - startTime < 50) {

      if (port.available()) {
        frame[index] = port.read();
        index++;
      }
    }

    if (index < 16) {
      return false;
    }

    if (frame[1] != 0x00 || frame[2] != 0xFF) {
      return false;
    }

    uint8_t checksum = 0;

    for (int i = 0; i < 15; i++) {
      checksum += frame[i];
    }

    if (checksum != frame[15]) {
      return false;
    }

    return true;
  }

  return false;
}

uint32_t getDistanceMM(uint8_t *frame) {

  return ((uint32_t)frame[8]) |
         ((uint32_t)frame[9] << 8) |
         ((uint32_t)frame[10] << 16);
}

void clearTOFSerialBuffers() {

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

// =====================
// TEST 4: IR SENSOR TEST
// =====================
void runIRTest() {

  if (!irMessagePrinted) {
    Serial.println("IR sensors ready");
    Serial.println("Higher value = darker / weaker reflection");
    Serial.println("Lower value  = lighter / stronger reflection");
    Serial.println();
    irMessagePrinted = true;
  }

  if (millis() - lastIRPrintTime >= 300) {

    readQTR_RC();

    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print("S");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(sensorValues[i]);

      if (i < SensorCount - 1) {
        Serial.print(" | ");
      }
    }

    Serial.println();

    lastIRPrintTime = millis();
  }
}

void readQTR_RC() {

  // Step 1: charge all sensor capacitors
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }

  delayMicroseconds(10);

  // Step 2: switch pins to input and measure discharge time
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorValues[i] = timeout;
  }

  unsigned long startTime = micros();
  bool allDone = false;

  while (!allDone && (micros() - startTime < timeout)) {
    allDone = true;

    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] == timeout) {
        if (digitalRead(sensorPins[i]) == LOW) {
          sensorValues[i] = micros() - startTime;
        } else {
          allDone = false;
        }
      }
    }
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
