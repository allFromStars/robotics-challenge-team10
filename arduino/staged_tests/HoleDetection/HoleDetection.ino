#include <Wire.h>
#include <Motoron.h>

// ======================================================
// MOTOR SETUP
// ======================================================

MotoronI2C motors(16);

const int LEFT_CH = 1;
const int RIGHT_CH = 2;

const int LEFT_POLARITY = 1;
const int RIGHT_POLARITY = -1;

// ======================================================
// SPEED SETTINGS
// ======================================================

const int CREEP_SPEED = -250;
const int WIGGLE_SPEED = 200;

// ======================================================
// SENSOR SETUP
// ======================================================

const uint8_t SensorCount = 9;

// IR sensor pins
const uint8_t sensorPins[SensorCount] =
{
  22, 23, 24, 25, 26, 27, 28, 29, 30
};

uint16_t sensorValues[SensorCount];

const uint16_t timeout = 2500;

// individual wood baselines
uint16_t woodBaselines[SensorCount];

// ======================================================
// QTR SENSOR READ
// ======================================================

void readQTR_RC()
{
  // Charge capacitors
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }

  delayMicroseconds(10);

  // Switch to input
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    pinMode(sensorPins[i], INPUT);
    sensorValues[i] = timeout;
  }

  unsigned long startTime = micros();

  bool allDone = false;

  while (!allDone &&
         (micros() - startTime < timeout))
  {
    allDone = true;

    for (uint8_t i = 0; i < SensorCount; i++)
    {
      if (sensorValues[i] == timeout)
      {
        if (digitalRead(sensorPins[i]) == LOW)
        {
          sensorValues[i] =
            micros() - startTime;
        }
        else
        {
          allDone = false;
        }
      }
    }
  }
}

// ======================================================
// STABLE SENSOR AVERAGE
// ======================================================

void readQTR_Stable()
{
  uint32_t sums[SensorCount] = {0};

  const int samples = 5;

  for (int s = 0; s < samples; s++)
  {
    readQTR_RC();

    for (int i = 0; i < SensorCount; i++)
    {
      sums[i] += sensorValues[i];
    }

    delay(3);
  }

  for (int i = 0; i < SensorCount; i++)
  {
    sensorValues[i] = sums[i] / samples;
  }
}

// ======================================================
// SURFACE CLASSIFICATION
// ======================================================

/*
0 = Wood
1 = Tape
2 = Empty Hole
3 = Seed Hole
*/

uint8_t getSurfaceState(uint8_t sensorIndex,
                        uint16_t rawValue)
{
  uint16_t woodMax =
    woodBaselines[sensorIndex] + 25;

  uint16_t emptyMax =
    woodBaselines[sensorIndex] + 55;

  uint16_t seedMax =
    woodBaselines[sensorIndex] + 130;

  if (rawValue < woodMax)
  {
    return 0;
  }
  else if (rawValue < emptyMax)
  {
    return 2;
  }
  else if (rawValue < seedMax)
  {
    return 3;
  }
  else
  {
    return 1;
  }
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);

  while (!Serial)
  {
    delay(10);
  }

  delay(1000);

  // ----------------------------------------
  // MOTOR INIT
  // ----------------------------------------

  Wire1.begin();

  motors.setBus(&Wire1);

  motors.reinitialize();
  motors.clearResetFlag();

  motors.setCommandTimeoutMilliseconds(1000);

  motors.setMaxAcceleration(LEFT_CH, 400);
  motors.setMaxDeceleration(LEFT_CH, 400);

  motors.setMaxAcceleration(RIGHT_CH, 400);
  motors.setMaxDeceleration(RIGHT_CH, 400);

  killMotors();

  // ----------------------------------------
  // SENSOR CALIBRATION
  // ----------------------------------------

  Serial.println("=== 9 SENSOR WOOD CALIBRATION ===");
  Serial.println("Place ALL sensors over wood");

  for (int i = 3; i > 0; i--)
  {
    Serial.print("Starting in ");
    Serial.println(i);

    delay(1000);
  }

  unsigned long sums[SensorCount] =
  {
    0,0,0,0,0,0,0,0,0
  };

  for (int sample = 0; sample < 20; sample++)
  {
    readQTR_RC();

    for (uint8_t i = 0; i < SensorCount; i++)
    {
      sums[i] += sensorValues[i];
    }

    delay(20);
  }

  Serial.println("\n--- BASELINES ---");

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    woodBaselines[i] = sums[i] / 20;

    Serial.print("Sensor ");
    Serial.print(i);

    Serial.print(": ");

    Serial.println(woodBaselines[i]);
  }

  Serial.println("-----------------\n");

  delay(1000);

  Serial.println("Robot starting...");
}

// ======================================================
// MAIN LOOP
// ======================================================

void loop()
{
  motors.resetCommandTimeout();

  // ----------------------------------------
  // CREEP FORWARD
  // ----------------------------------------

  setMotorSpeeds(CREEP_SPEED,
                 CREEP_SPEED);

  readQTR_Stable();

  // ----------------------------------------
  // COUNT EMPTY HOLE DETECTIONS
  // ----------------------------------------

  int emptyCount = 0;

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    uint8_t state =
      getSurfaceState(i,
                      sensorValues[i]);

    if (state == 2)
    {
      emptyCount++;
    }
  }

  // ----------------------------------------
  // HOLE FOUND
  // ----------------------------------------

  if (emptyCount >= 2)
  {
    killMotors();

    Serial.println("\nEMPTY HOLE DETECTED");

    delay(200);

    executeAlignment();
  }
}

// ======================================================
// ALIGNMENT ROUTINE
// ======================================================

void executeAlignment()
{
  const float ARRAY_CENTER = 4.0;

  const float TOLERANCE = 0.35;

  int stableHits = 0;

  const int REQUIRED_HITS = 8;

  while (stableHits < REQUIRED_HITS)
  {
    motors.resetCommandTimeout();

    readQTR_Stable();

    // ----------------------------------------
    // FIND HOLE REGION
    // ----------------------------------------

    int leftBound = -1;
    int rightBound = -1;

    for (int i = 0; i < SensorCount; i++)
    {
      uint8_t state =
        getSurfaceState(i,
                        sensorValues[i]);

      if (state == 2)
      {
        if (leftBound == -1)
        {
          leftBound = i;
        }

        rightBound = i;
      }
    }

    // ----------------------------------------
    // LOST HOLE
    // ----------------------------------------

    if (leftBound == -1)
    {
      stableHits = 0;

      // slow search rotation
      setMotorSpeeds(-90, 90);

      delay(20);

      continue;
    }

    // ----------------------------------------
    // CALCULATE HOLE CENTER
    // ----------------------------------------

    float holeCenter =
      (leftBound + rightBound) / 2.0;

    float error =
      ARRAY_CENTER - holeCenter;

    Serial.print("Hole Center: ");
    Serial.print(holeCenter);

    Serial.print(" Error: ");
    Serial.println(error);

    // ----------------------------------------
    // CENTERED
    // ----------------------------------------

    if (abs(error) <= TOLERANCE)
    {
      killMotors();

      stableHits++;

      delay(30);

      continue;
    }

    stableHits = 0;

    // ----------------------------------------
    // WIGGLE ALIGNMENT
    // ----------------------------------------

    if (error > 0)
    {
      // hole is LEFT
      setMotorSpeeds(-WIGGLE_SPEED,
                      WIGGLE_SPEED);
    }
    else
    {
      // hole is RIGHT
      setMotorSpeeds(WIGGLE_SPEED,
                    -WIGGLE_SPEED);
    }

    delay(18);
  }

  // ==================================================
  // FINISHED
  // ==================================================

  killMotors();

  Serial.println("\nHOLE CENTERED");
  Serial.println("Alignment complete");

  while (true)
  {
    killMotors();
    delay(1000);
  }
}

// ======================================================
// MOTOR FUNCTIONS
// ======================================================

void setMotorSpeeds(int leftSpeed,
                    int rightSpeed)
{
  motors.setSpeed(
    LEFT_CH,
    LEFT_POLARITY * leftSpeed
  );

  motors.setSpeed(
    RIGHT_CH,
    RIGHT_POLARITY * rightSpeed
  );
}

void killMotors()
{
  motors.setSpeed(LEFT_CH, 0);
  motors.setSpeed(RIGHT_CH, 0);
}