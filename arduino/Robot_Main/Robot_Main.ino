#include <Wire.h>
#include <Motoron.h>
#include "ICM_20948.h"
#include <MFRC522_I2C.h>

enum RobotState {
  STATE_STANDBY_BASE,       // Parked at base waiting for start cue
  STATE_BASE_NAVIGATION,

  STATE_RAMP,

  STATE_PLAN, 

  STATE_TURN,         // Go back to plan
  STATE_ALIGNCOMPASS,

  STATE_NAVIGATING_LINES,     // Tracking line in arena
  STATE_NAVIGATING_OPEN,      // Navigating in open field

  STATE_ALIGN_SEED,         // After wanted RFID detected, creep forward and use reflectance to align itself slowly
  STATE_PLANTING,           // Paused over an RFID seed hole, activating the hopper mechanism

  STATE_RESCUE_MODE,        // received signal to save robot that is right infront, tap robot
  STATE_STRANDED_ALIVE,     // Stop wheels activate LED, do stranded protocol

  STATE_REVIVED_RETURN,     // Revived - do revival protocol
  STATE_EMERGENCY_STOP,      // Hardware kill switch fallback state

  STATE_EXIT_ARENA,

  STATE_AIRLOCK_B,

  STATE_DEBUG,
};

RobotState currentState = STATE_DEBUG;



bool imuOnline   = false;
bool motorOnline = false;
bool rfidOnline  = false;
bool tofFrontOnline  = false;
bool tofLeftOnline   = false;
bool tofRight1Online = false;
bool tofRight2Online = false;


MotoronI2C mc(16);
int maxAccel = 150;
int maxDecel = 250;
const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;
const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;

// --- RFID Setup ---
#define RFID_ADDR 0x28
MFRC522_I2C rfid(RFID_ADDR, -1, &Wire1);
bool rfidCardPresent = false;

// --- TOF Distance Sensor Setup ---
const unsigned long SENSOR_BAUD = 921600;
const int TOF_FILTER_SIZE = 10;
int32_t offset1 = 0, offset2 = 0, offset3 = 0, offset4 = 0; 

// --- IR Reflectance Array Setup ---
const uint8_t IR_COUNT = 9;
const uint8_t irPins[IR_COUNT] = { 22, 23, 24, 25, 26, 27, 28, 29, 30 };
const uint16_t IR_TIMEOUT_MICROS = 2500;

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
  Coordinate currentPos;  // Holds your active x and y
  Coordinate destination; // Holds your target x and y
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
  robotInfo.seedsLeft = 5;      // Start with a full hopper
  

  robotInfo.currentPos.x = -1;  //at base
  robotInfo.currentPos.y = -1;
  

  robotInfo.destination.x = 5;
  robotInfo.destination.y = 5;



}

void loop() {
  //DebugSensors();

  refreshAllSensors();


  switch (currentState) {

    case STATE_STANDBY_BASE:
      stopMotors();
      break;

    case STATE_BASE_NAVIGATION:
      break;

    case STATE_RAMP:
      break;

    case STATE_PLAN:
      break;

    case STATE_TURN:
      break;

    case STATE_ALIGN_SEED:
      break;

    case STATE_PLANTING:
      break;

    case STATE_STRANDED_ALIVE:
      stopMotors();
      break;

    case STATE_REVIVED_RETURN:
      break;

    case STATE_EMERGENCY_STOP:
      stopMotors();
      break;
    
    case STATE_EXIT_ARENA:
      break;

    case STATE_AIRLOCK_B:
      break;

    case STATE_DEBUG:
      DebugSensors();
      break;

    default:
      stopMotors();
      break;
  }

  // run serial 4 times per sec
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 250) {
    lastPrint = millis();

  }
}

