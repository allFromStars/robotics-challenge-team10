#include <Wire.h>
#include <Motoron.h>
#include "ICM_20948.h"
#include <MFRC522_I2C.h>
#include "config.h"
#include "pins.h"
#include "robot_state.h"

bool debugIRMode = false;

RobotState currentState = STATE_DEBUG; 

bool imuOnline   = false;
bool motorOnline = false;
bool rfidOnline  = false;
bool tofFrontOnline  = false;
bool tofLeftOnline   = false;
bool tofRight1Online = false;
bool tofRight2Online = false;



MotoronI2C mc(16);

// --- RFID Setup ---
MFRC522_I2C rfid(RFID_ADDR, -1, &Wire1);
bool rfidCardPresent = false;

// --- TOF Distance Sensor Setup ---

// --- IR Reflectance Array Setup ---

// --- IMU Setup ---
ICM_20948_I2C myICM;
float gyroBiasZ = 0;
unsigned long lastTimeMicros = 0;

// Shared Global Storage Struct
struct SENSORS {
  float yaw;
  float compassNorth;
  uint32_t tof_front;
  uint32_t tof_left;
  uint32_t tof_right1;
  uint32_t tof_right2;
  uint16_t irLineArray[IR_COUNT];
  uint32_t rfidInfo;
}; 
SENSORS sensors;

struct Coordinate {
  int x;
  int y;
};

struct INFO {
  int seedsLeft;
  Coordinate currentPos;  
  Coordinate destination;
};
INFO robotInfo;


bool initSensors();
bool initMotors();
bool checkI2CDevice(TwoWire &wireBus, uint8_t address);
void refreshAllSensors();

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } 
  delay(500);
  
  Serial.println("\n=========================================");
  Serial.println("       Components test   ");
  Serial.println("=========================================");


  Wire.begin();
  Wire.setClock(400000);
  Wire1.begin();
  Wire1.setClock(400000);
  delay(500); 


  Serial1.begin(SENSOR_BAUD);
  Serial2.begin(SENSOR_BAUD);
  Serial3.begin(SENSOR_BAUD);
  Serial4.begin(SENSOR_BAUD);

  // Initialise Reflectance
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(irPins[i], INPUT);
  }

  // Initialise Encoders
  pinMode(LEFT_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN_B, INPUT_PULLUP);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_A), readLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_A), readRightEncoder, CHANGE);

  imuOnline   = initIMU();
  motorOnline = initMotors();
  rfidOnline  = checkI2CDevice(Wire1, RFID_ADDR);
  
  if (rfidOnline) {
    rfid.PCD_Init();
  }

  // check if data is coming from tof for 200 ms
  unsigned long serialCheckStart = millis();
  while (millis() - serialCheckStart < 200) {
    if (Serial1.available() > 0) tofFrontOnline  = true;
    if (Serial2.available() > 0) tofLeftOnline   = true;
    if (Serial3.available() > 0) tofRight1Online = true;
    if (Serial4.available() > 0) tofRight2Online = true;
  }

  // =============================================================================
  // Components check
  // =============================================================================
  Serial.println(imuOnline   ? "[OK] IMU Sensor"  : "[FAILED] IMU Sensor");
  Serial.println(motorOnline ? "[OK] Motoron MCU" : "[FAILED] Motoron MCU");
  Serial.println(rfidOnline  ? "[OK] RFID Reader" : "[FAILED] RFID Reader");
  
  // print tof status
  Serial.println(tofFrontOnline  ? "[OK] TOF Front Sensor"  : "[FAILED] TOF Front Sensor");
  Serial.println(tofLeftOnline   ? "[OK] TOF Left Sensor"   : "[FAILED] TOF Left Sensor");
  Serial.println(tofRight1Online ? "[OK] TOF Right 1 Sensor" : "[FAILED] TOF Right 1 Sensor");
  Serial.println(tofRight2Online ? "[OK] TOF Right 2 Sensor" : "[FAILED] TOF Right 2 Sensor");
  Serial.println("=========================================\n");

  //robot info setup
  robotInfo.seedsLeft = STARTING_SEED_COUNT;      // Start with a full hopper
  

  robotInfo.currentPos.x = START_X;  //at base
  robotInfo.currentPos.y = START_Y;
  

  robotInfo.destination.x = DEFAULT_DESTINATION_X;
  robotInfo.destination.y = DEFAULT_DESTINATION_Y;
  


}

void loop() {
  refreshAllSensors();

  if (digitalRead(BUTTON_PIN) == LOW) { 
    currentState = STATE_EMERGENCY_STOP;
  }

  if (Serial.available() > 0) {
      char cmd = Serial.read();
      
      if (cmd == 'g' || cmd == 'G') {
        // ... go logic
      }
      else if (cmd == 'd' || cmd == 'D') {
        debugIRMode = false; // Ensure it enters standard debug
        currentState = STATE_DEBUG;
        Serial.println("--- Entering Standard Debug Mode ---");
      }
      else if (cmd == 'i' || cmd == 'I') {
        debugIRMode = true; // Swap to IR debug
        currentState = STATE_DEBUG;
        Serial.println("--- Entering Full IR Debug Mode ---");
      }
      else if (cmd == 'c' || cmd == 'C') {
        Serial.println("--- Manual Calibration Triggered ---");
        stopMotors(); 
        startIRCalibration(); 
        currentState = STATE_CALIBRATING_IR; 
      }
      else if (cmd == 'p' || cmd == 'P') {
        Serial.println("PLANTING SEQUENCE");
        startPlanting(); 
        currentState = STATE_PLANTING; 
      }
  }


  switch (currentState) {

    case STATE_STANDBY_BASE:
      stopMotors();
      break;

    case STATE_CALIBRATING_IR:
      // Run the non-blocking math
      updateIRCalibration();
      
      // When 12 seconds are up, transition to standby!
      if (isCalibrationComplete()) {
        Serial.println("Robot Ready. Waiting for 'g' command.");
        currentState = STATE_STANDBY_BASE;
      }
      break;


    case STATE_NAVIGATING_LINES:

      updateTask3LineNavigation(); 

      if (task3LineNavigationComplete()) {
         Serial.println("SUCCESS: Reached target nodes!");
         currentState = STATE_STANDBY_BASE; 
      } 
      else if (task3LineNavigationFailed()) {
         Serial.println("ERROR: Failsafe triggered (Lost line or timeout).");
         currentState = STATE_EMERGENCY_STOP; 
      }
      break;

    case STATE_EMERGENCY_STOP:
      stopMotors();
      break;

    case STATE_DEBUG:
      DebugSensors();
      break;

    case STATE_TURN:
      if (updateTurnAngle()) {
          // Turn is complete! Transition to next state...
        }
      break;
    case STATE_PLANTING:
      // call non blocking planter
      if (updatePlanting()) {
        // The planter returned TRUE - has finished
        
        robotInfo.seedsLeft--; // decrease seeds
        Serial.print("Seeds remaining: ");
        Serial.println(robotInfo.seedsLeft);
        currentState = STATE_PLAN; // Go to next target
      }
      break;

    case STATE_BASE_NAVIGATION:
    case STATE_RAMP:
    case STATE_PLAN:

    case STATE_ALIGN_SEED:

    case STATE_STRANDED_ALIVE:
    case STATE_REVIVED_RETURN:
    case STATE_EXIT_ARENA:
    case STATE_AIRLOCK_B:
      stopMotors(); 
      break;

    default:
      stopMotors();
      break;
  }
  /*
    // run serial 4 times per sec
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 250) {
    lastPrint = millis();

  }
  */
}




