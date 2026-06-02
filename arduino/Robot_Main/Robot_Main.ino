#include <Wire.h>
#include <Motoron.h>
#include "ICM_20948.h"
#include <MFRC522_I2C.h>
#include <MiniMessenger.h>
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
  Coordinate currentPos;       // 'R' on the map
  Coordinate finalDestination;  // 'T' on the map
  Coordinate waypoints[30];    // '*' on the map
  int totalWaypoints;
  int currentWaypointIdx;
  Coordinate getNextNode() {
    if (currentWaypointIdx < totalWaypoints) {
      return waypoints[currentWaypointIdx];
    }
    return finalDestination; 
  }
};
INFO robotInfo;

const int TOTAL_SEED_TARGETS = 5; 
Coordinate seedTargets[TOTAL_SEED_TARGETS] = {
  {0, 2}, // Target 1
  {1, 2}, // Target 2
  {0, 0}, // Target 3
  {4, 6}, // Target 4
  {6, 6}  // Target 5
};

int currentTargetIndex = 0; 
bool headingToExit = false; 
const float AIRLOCK_HEADING = -90;


bool initSensors();
bool initMotors();
void stopMotors();
void abortPlanting();
bool checkI2CDevice(TwoWire &wireBus, uint8_t address);
void refreshAllSensors();
bool calibrateGyroBiasZ(unsigned long calibrationMs);

void startRampWallFollowing();
int updateRampWallFollowing(bool trackLeftWall);
void initRobotCommunication();
void updateRobotCommunication();
bool robotAllowedToMove();
bool robotSafetyEmergencyActive();
bool serverApiRequired();
bool requestOpenAirlock(char airlock, uint32_t tagId);
bool airlockRequestAccepted();
bool airlockRequestRejected();
bool requestFertilityCheck(uint32_t tagId);
bool fertilityReplyReady();
bool currentTagIsPlantable();
int fertilityReplyCoordinateX();
int fertilityReplyCoordinateY();
bool reportSeedPlanted(uint32_t tagId);
bool requestRevive(int targetTeam, const char *targetBoard);
bool reviveReplyReady();
bool reviveSucceeded();
bool requestOccupancyMap();
bool occupancyMapIsReady();
uint8_t getOccupancyMapCell(int x, int y);
float normaliseAngleDeg(float angle);

bool robotStateRequiresSafety(RobotState state) {
  switch (state) {
    case STATE_CALIBRATING_IR:
    case STATE_STANDBY_BASE:
    case STATE_AIRLOCK_B:
    case STATE_DEBUG:
    case STATE_EMERGENCY_STOP:
      return false;
    default:
      return true;
  }
}

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
  pinMode(BUMPER_PIN, INPUT_PULLUP);

  // RGB LED Setup (Common Anode) ---
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  
  // HIGH = OFF, LOW = ON
  digitalWrite(RGB_RED_PIN, HIGH); 
  digitalWrite(RGB_GREEN_PIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_A), readLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_A), readRightEncoder, CHANGE);

  imuOnline   = initIMU();
  motorOnline = initMotors();
  rfidOnline  = checkI2CDevice(Wire1, RFID_ADDR);
  
  if (rfidOnline) {
    rfid.PCD_Init();
  }

  // =============================================================================
  // Components check
  // =============================================================================
  Serial.println(imuOnline   ? "[OK] IMU Sensor"  : "[FAILED] IMU Sensor");
  Serial.println(motorOnline ? "[OK] Motoron MCU" : "[FAILED] Motoron MCU");
  Serial.println(rfidOnline  ? "[OK] RFID Reader" : "[FAILED] RFID Reader");
  
  Serial.println("[INFO] TOF sensors: runtime UART frame detection enabled");
  Serial.println("=========================================\n");

  //robot info setup
  robotInfo.seedsLeft = STARTING_SEED_COUNT;      // Start with a full hopper
  

  robotInfo.currentPos.x = START_X;  //at base
  robotInfo.currentPos.y = START_Y;
  

  robotInfo.finalDestination.x = DEFAULT_DESTINATION_X;
  robotInfo.finalDestination.y = DEFAULT_DESTINATION_Y;
  
  robotInfo.finalDestination = seedTargets[0]; // Load the first target from the list!

  initRobotCommunication();

}

void loop() {
  refreshAllSensors();
  updateRobotCommunication();
  mc.resetCommandTimeout();

  if (robotSafetyEmergencyActive()) {
    currentState = STATE_EMERGENCY_STOP;
  }

  if (Serial.available() > 0) {
      char cmd = Serial.read();
      
      if (cmd == 'g' || cmd == 'G') {
        stopMotors();
        calibrateGyroBiasZ(1000);
        sensors.yaw = 0.0;
        lastTimeMicros = micros();
        newPathNeeded = true;
        currentState = STATE_PLAN;
        Serial.println("--- Starting Mission Plan ---");
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
        if (!robotAllowedToMove()) {
          Serial.println("PLANTING BLOCKED: enable WiFi and mechanical switch first.");
          return;
        }

        Serial.println("PLANTING SEQUENCE");
        startPlanting(); 
        currentState = STATE_PLANTING; 
      }
      else if (cmd == 'r' || cmd == 'R') {
        Serial.println("RAMP");
        startRampWallFollowing(); 
        currentState = STATE_RAMP; 
      }
      else if (cmd == 's' || cmd == 'S') {
        Serial.println("SAVING RESCUE MODE");
        startRampWallFollowing(); 
        currentState = STATE_RESCUE_MODE; 
      }
  }

  if (!robotAllowedToMove() && robotStateRequiresSafety(currentState)) {
    stopMotors();

    if (currentState == STATE_PLANTING) {
      abortPlanting();
    }

    return;
  }

  switch (currentState) {

    case STATE_PLAN: {
      printVisualMap();

      // check if we have finished
      if (currentTargetIndex >= TOTAL_SEED_TARGETS || robotInfo.seedsLeft <= 0) {
        if (!headingToExit) {
          Serial.println("Mission Complete or Out of Seeds! Locking Exit Coordinate.");
          headingToExit = true;                // finished
          robotInfo.finalDestination = {0, 7}; // airlock exit coords
          newPathNeeded = true;
        }
      } else {
        // Load the next target from the list
        robotInfo.finalDestination = seedTargets[currentTargetIndex]; 
      }

      // check if current position is destination
      if (robotInfo.currentPos.x == robotInfo.finalDestination.x && robotInfo.currentPos.y == robotInfo.finalDestination.y) {
        
        // check if we are heading for exit or planting
        if (headingToExit) {
          Serial.println("Arrived at the Exit Airlock. Initiating exit Sequence...");
          
          // Calculate the turn to face the correct way for the airlock
          float turnAngle = normaliseAngleDeg(AIRLOCK_HEADING - sensors.yaw);
          startTurnAngle(turnAngle);
          
          currentState = STATE_ALIGN_AIRLOCK_B;
          
        } else {
          Serial.println("Target reached! Deploying Planter...");
          startPlanting();
          currentState = STATE_PLANTING; 
        }
        break; 
      }

      // if new path is required, calculate, if not increment waypoint
      if (newPathNeeded) {
        if (!calculateWeightedPath()) {
          Serial.println("ERROR: No valid path to target.");
          currentState = STATE_EMERGENCY_STOP;
          break;
        }
        robotInfo.currentWaypointIdx = 0;
        newPathNeeded = false;
      } else {
        robotInfo.currentWaypointIdx += 1;
      }

      // pass over to turn
      Coordinate nextNode = robotInfo.getNextNode(); 
      float targetHeading = getTargetHeading(robotInfo.currentPos, nextNode);
      float turnAngle = normaliseAngleDeg(targetHeading - sensors.yaw);
      Serial.print("[PLAN] Next node target heading: ");
      Serial.print(targetHeading, 1);
      Serial.print(" current yaw: ");
      Serial.print(sensors.yaw, 1);
      Serial.print(" turn angle: ");
      Serial.println(turnAngle, 1);
      
      startTurnAngle(turnAngle); 
      currentState = STATE_TURN;
      break;
    }


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


    case STATE_NAVIGATING_LINES: {
      updateTask3LineNavigation(); 

      if (task3LineNavigationComplete()) {
        Serial.println("SUCCESS: Reached next node!");
        robotInfo.currentPos = robotInfo.getNextNode(); // Update Map Memory
        //YawToGrid();                                // EXPERIMENTAL round the gyro reading to snap to nearest 90 degrees while we are navigating grid lines
         
        currentState = STATE_PLAN; 

      } 
      else if (task3LineNavigationFailed()) {
         Serial.println("ERROR: Failsafe triggered (Lost line or timeout).");
         Serial.print("Task 3 failure reason: ");
         Serial.println(getTask3LineNavigationFailureReason());
         currentState = STATE_EMERGENCY_STOP; 
         // currentState = STATE_PLAN;
      }
      break;
    }
    case STATE_NAVIGATING_OPEN: 
      updateTask4OpenNavigation();

      if (task4OpenNavigationComplete()) {
        Serial.println("SUCCESS: Reached next open-field node!");
        robotInfo.currentPos = robotInfo.getNextNode();
        snapYawToGrid();
        currentState = STATE_PLAN;
      }
      else if (task4OpenNavigationFailed()) {
        Serial.println("ERROR: Open-field navigation failed.");
        currentState = STATE_EMERGENCY_STOP;
      }
      
      break;

    
    case STATE_TURN:{
      Coordinate nextNode = robotInfo.getNextNode();

      if (updateTurnAngle()) {

        const int OBSTACLE_THRESHOLD_MM = 250;
        refreshAllSensors();

        if (sensors.tof_front < OBSTACLE_THRESHOLD_MM && sensors.tof_front > 0) {
            Serial.print("OBSTACLE DETECTED at distance: ");
            Serial.println(sensors.tof_front);
            
            // Mark the target node as an impassable wall (3) in the digital map
            arenaMap[8 - nextNode.y][nextNode.x] = 3; 
            
            // Tell the brain to map a new route from safe spot
            newPathNeeded = true;
            currentState = STATE_PLAN;
            
            break; // Break out immediately so we don't start driving!
        }

        byte nextTerrain = arenaMap[8 - nextNode.y][nextNode.x]; 
        if (nextTerrain == 1) {
            bool arrivingForPlanting =
              !headingToExit &&
              nextNode.x == robotInfo.finalDestination.x &&
              nextNode.y == robotInfo.finalDestination.y;

            startTask3LineNavigation(arrivingForPlanting ? LINE_ARRIVE_FOR_PLANTING : LINE_ARRIVE_FOR_TURN);
            currentState = STATE_NAVIGATING_LINES;
        } else {
            startTask4OpenNavigation();
            currentState = STATE_NAVIGATING_OPEN;
        }
      }
      break;
    }
    case STATE_PLANTING:
      // call update planting after initial call
      if (updatePlanting()) {
        // The planter returned TRUE - has finished
        // Seed count is decreased inside of seed function in Hardware.ino
        Serial.print("Seeds remaining: ");
        Serial.println(robotInfo.seedsLeft);
        currentTargetIndex++; //move to next target in the list
        startPostPlantAdvance();
        currentState = STATE_ADVANCE_AFTER_PLANTING;
      }
      break;
    case STATE_ADVANCE_AFTER_PLANTING:
      if (updatePostPlantAdvance()) {
        newPathNeeded = true; //tell plan to calculate route
        currentState = STATE_PLAN; // Go to next target
      }
      break;
    case STATE_EMERGENCY_STOP:
      stopMotors();
      abortPlanting();
      break;
    case STATE_BASE_NAVIGATION:

    case STATE_RAMP: {
      // Returns 1 when it successfully crosses the RFID scanner
      if (updateRampWallFollowing(true) == 1) { 
        Serial.println("SUCCESS: Cleared the ramp and entered the arena!");
        
        Coordinate truePosition;// = getCoordinateFromRFID(sensors.rfidInfo);
        truePosition.x = 0;
        truePosition.y = 3; //hard coded airlock coordinates
        if (truePosition.x != -1 && truePosition.y != -1) {
          robotInfo.currentPos = truePosition;
          Serial.print("[RAMP EXIT] Synced to: (X:");
          Serial.print(robotInfo.currentPos.x); Serial.print(", Y:");
          Serial.print(robotInfo.currentPos.y); Serial.println(")");
        }

        snapYawToGrid(); 
        newPathNeeded = true;
        currentState = STATE_PLAN; 
      }
      break;
    }
    case STATE_RESCUE_MODE: {
      // Static memory for the non-blocking hold
      static bool rescueContactMade = false;
      static unsigned long contactTimeMs = 0;

      //Approach the target
      if (!rescueContactMade) {
        int rescueStatus = updateRescueApproach();
        
        if (rescueStatus == 1) {
          // contact - Mark the time and trigger the LED
          rescueContactMade = true;
          contactTimeMs = millis();
          
          // turn on LED
          digitalWrite(RGB_GREEN_PIN, LOW); 
          Serial.println("[RESCUE] Contact made! Illuminating GREEN LED and holding for 2s...");
        } 
        else if (rescueStatus == -1) {
          Serial.println("ABORT: Rescue timeout triggered.");
          currentState = STATE_EMERGENCY_STOP; 
        }
      } 
      

      else {

        stopMotors(); 
        
        if (millis() - contactTimeMs >= 2000) {
          // Turn OFF Green LED 
          digitalWrite(RGB_GREEN_PIN, HIGH);
          digitalWrite(RGB_RED_PIN, HIGH);
          //digitalWrite(RGB_BLUE_PIN, HIGH);
          
          Serial.println("[RESCUE] 2-second hold complete. Initiating return sequence.");
          
          robotInfo.finalDestination = {START_X, START_Y}; 
          newPathNeeded = true;
          

          rescueContactMade = false; 
          
          currentState = STATE_EMERGENCY_STOP; 
        }
      }
      break;
    }

/*
    case STATE_STRANDED_ALIVE:
    case STATE_REVIVED_RETURN:
    case STATE_EXIT_ARENA:
 */

    case STATE_ALIGN_AIRLOCK_B:  //finish rotating towards airlock
      if (updateTurnAngle()) {
        Serial.println("Safely parked in the airlock!");
        currentState = STATE_AIRLOCK_B; 
      }
      break;

    case STATE_AIRLOCK_B:
      stopMotors(); 
      break;

    case STATE_DEBUG:
      DebugSensors();
      break;

    default:
      stopMotors();
      break;
  }

}
