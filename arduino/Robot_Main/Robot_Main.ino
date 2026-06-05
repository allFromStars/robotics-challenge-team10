#include <Wire.h>
#include <Motoron.h>
#include "ICM_20948.h"
#include <MFRC522_I2C.h>
#include <MiniMessenger.h>
#include "config.h"
#include "pins.h"
#include "robot_state.h"


// true = Grid Area priority, false = Open Area priority
const int TOTAL_STRATEGY_STEPS = 5;
bool strategyPlaybook[TOTAL_STRATEGY_STEPS] = {true, true, false, true, false};
int currentStrategyIdx = 0;



// 9x9 => max = 8
const int MAX_GRID_INDEX = 8; 

// --- Internal Map Landmarks ---
const int INTERNAL_AIRLOCK_A_X = 0;
const int INTERNAL_AIRLOCK_A_Y = 2; // ENTRANCE (Calibrate here)

const int INTERNAL_AIRLOCK_B_X = 0;
const int INTERNAL_AIRLOCK_B_Y = 6; // EXIT (Drive here to finish)



bool debugIRMode = false;
bool startPlanWhenAllowed = false;

RobotState currentState = STATE_DEBUG; 



bool imuOnline   = false;
bool motorOnline = false;
bool rfidOnline  = false;
bool tofFrontOnline  = false;
bool tofLeftOnline   = false;
bool tofRight1Online = false;
bool tofRight2Online = false;

extern volatile long leftEncoderPos;
extern volatile long rightEncoderPos;


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

/*
const int TOTAL_SEED_TARGETS = 5; 
Coordinate seedTargets[TOTAL_SEED_TARGETS] = {
  {1, 4}, // Target 1
  {1, 2}, // Target 2
  {0, 0}, // Target 3
  {4, 6}, // Target 4
  {6, 6}  // Target 5
};
robotInfo.finalDestination = seedTargets[0]; // Load the first target from the list!

int currentTargetIndex = 0; 
*/

bool headingToExit = false; 
const float AIRLOCK_HEADING = 180;
const bool ENABLE_SERIAL_COMMANDS = false;


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
bool consumeMechanicalStartRequest();
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
void setRescueLedOverride(bool enabled);
float normaliseAngleDeg(float angle);
void readLeftEncoder();
void readRightEncoder();
bool initIMU();
void driveMotors(int leftLogicalSpeed, int rightLogicalSpeed);
void startTurnAngle(float targetAngle);
bool updateTurnAngle();
void processLinePosition();
void followGridLinePD(int baseSpeed);
void driveForwardMagnitudes(int leftMagnitude, int rightMagnitude);
void startIRCalibration();
void updateIRCalibration();
bool isCalibrationComplete();
void saveIRCalibrationToMemory();
void cancelTurnAngle();
void startTask2BaseExit();
int updateTask2BaseExit();

void resetMissionProgress() {
  stopMotors();
  abortPlanting();

  robotInfo.seedsLeft = STARTING_SEED_COUNT;
  robotInfo.currentPos.x = START_X;
  robotInfo.currentPos.y = START_Y;
  robotInfo.finalDestination.x = DEFAULT_DESTINATION_X;
  robotInfo.finalDestination.y = DEFAULT_DESTINATION_Y;
  robotInfo.totalWaypoints = 0;
  robotInfo.currentWaypointIdx = 0;

  currentStrategyIdx = 0;
  headingToExit = false;
  newPathNeeded = true;
  startPlanWhenAllowed = false;

  sensors.yaw = 0.0;
  lastTimeMicros = micros();
}

void startFullMissionFromBase() {
  resetMissionProgress();
  if (SKIP_TASK2_BASE_TO_RAMP) {
    startRampWallFollowing();
    currentState = STATE_RAMP;
  } else {
    startTask2BaseExit();
    currentState = STATE_TASK2_BASE_EXIT;
  }
}

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
  delay(200);
  
  //serial.println("\n=========================================");
  //serial.println("       Components test   ");
  //serial.println("=========================================");


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

  //attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_A), readLeftEncoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_A), readRightEncoder, CHANGE);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_A), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_A), readRightEncoder, RISING);

  motorOnline = initMotors();
  stopMotors();

  //serial.println("[SYSTEM] Waiting 5 seconds before automatic calibration.");
  unsigned long calibrationDelayStartMs = millis();
  while (millis() - calibrationDelayStartMs < STARTUP_CALIBRATION_DELAY_MS) {
    if (motorOnline) {
      mc.resetCommandTimeout();
    }
    delay(10);
  }

  imuOnline   = initIMU();
  rfidOnline  = checkI2CDevice(Wire1, RFID_ADDR);
  
  if (rfidOnline) {
    rfid.PCD_Init();
  }

  // =============================================================================
  // Components check
  // =============================================================================
  //serial.println(imuOnline   ? "[OK] IMU Sensor"  : "[FAILED] IMU Sensor");
  //serial.println(motorOnline ? "[OK] Motoron MCU" : "[FAILED] Motoron MCU");
  //serial.println(rfidOnline  ? "[OK] RFID Reader" : "[FAILED] RFID Reader");
  
  //serial.println("[INFO] TOF sensors: runtime UART frame detection enabled");
  //serial.println("=========================================\n");

  //robot info setup
  robotInfo.seedsLeft = STARTING_SEED_COUNT;      // Start with a full hopper
  

  robotInfo.currentPos.x = START_X;  //at base
  robotInfo.currentPos.y = START_Y;
  

  robotInfo.finalDestination.x = DEFAULT_DESTINATION_X;
  robotInfo.finalDestination.y = DEFAULT_DESTINATION_Y;
  


  initRobotCommunication();
  //serial.println("[SYSTEM] Starting automatic IR calibration.");
  startIRCalibration();
  currentState = STATE_CALIBRATING_IR;
}
  


void loop() {

  yield();
  refreshAllSensors();
  updateRobotCommunication();
  if (motorOnline) {
      mc.resetCommandTimeout();
  }

  if (robotSafetyEmergencyActive()) {
    currentState = STATE_EMERGENCY_STOP;
  }

  if (currentState != STATE_CALIBRATING_IR && consumeMechanicalStartRequest()) {
    startFullMissionFromBase();
  }

  if (ENABLE_SERIAL_COMMANDS && Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == 'g' || cmd == 'G') {
      // Guard check for missing hardware
      if (!imuOnline) {
        //serial.println("[ERROR] Cannot calibrate gyro: IMU is OFFLINE!");
        return; // Erits loop() safely if the sensor is missing
      }

      // This mission start logic now sits properly inside the 'g' command block
      stopMotors();
      calibrateGyroBiasZ(1000);
      sensors.yaw = 0.0;
      lastTimeMicros = micros();
      startTask2BaseExit();
      currentState = STATE_TASK2_BASE_EXIT;
      //serial.println("--- Starting Task 2 Base Exit ---");
    }
    else if (cmd == 'd' || cmd == 'D') {
      debugIRMode = false; // Ensure it enters standard debug
      currentState = STATE_DEBUG;
      //serial.println("--- Entering Standard Debug Mode ---");
    }
    else if (cmd == 'i' || cmd == 'I') {
      debugIRMode = true; // Swap to IR debug
      currentState = STATE_DEBUG;
      //serial.println("--- Entering Full IR Debug Mode ---");
    }
    else if (cmd == 'c' || cmd == 'C') {
      //serial.println("--- Manual Calibration Triggered ---");
      stopMotors(); 
      startIRCalibration(); 
      currentState = STATE_CALIBRATING_IR; 
    }
    else if (cmd == 'p' || cmd == 'P') {
      if (!robotAllowedToMove()) {
        //serial.println("PLANTING BLOCKED: enable WiFi and mechanical switch first.");
        return;
      }

      //serial.println("PLANTING SEQUENCE");
      startPlanting(); 
      currentState = STATE_PLANTING; 
    }
    else if (cmd == 'r' || cmd == 'R') {
      //serial.println("RAMP");
      startRampWallFollowing(); 
      currentState = STATE_RAMP; 
    }
    else if (cmd == 's' || cmd == 'S') {
      //serial.println("SAVING RESCUE MODE");
      startRampWallFollowing(); 
      currentState = STATE_RESCUE_MODE; 
    }
    else if (cmd == 't' || cmd == 'T') {  
      //serial.println("\n--- [TEST] MOCK SERVER COORDINATE TEST ---");

      // 1. Pretend the server just told us a tag is at X: 2, Y: 8
      int mockServerX = 9;
      int mockServerY = 3;

//       Serial.print("Server Raw Input: (X: ");
//       Serial.print(mockServerX); Serial.print(", Y: "); Serial.print(mockServerY); //serial.println(")");

      // 2. Run it through your conversion function
      Coordinate translated = convertServerToInternal(mockServerX, mockServerY);

      // 3. Print the result that the pathfinder will actually use
//       Serial.print("Robot Translated: (X: ");
//       Serial.print(translated.x); Serial.print(", Y: "); Serial.print(translated.y); //serial.println(")");
      
      //serial.println("------------------------------------------");

      // 1. Pretend the server just told us a tag is at X: 2, Y: 8
      mockServerX = 8;
      mockServerY = 7;

//       Serial.print("Server Raw Input: (X: ");
//       Serial.print(mockServerX); Serial.print(", Y: "); Serial.print(mockServerY); //serial.println(")");

      // 2. Run it through your conversion function
      translated = convertServerToInternal(mockServerX, mockServerY);

      // 3. Print the result that the pathfinder will actually use
//       Serial.print("Robot Translated: (X: ");
//       Serial.print(translated.x); Serial.print(", Y: "); Serial.print(translated.y); //serial.println(")");
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

      // Check if the mission is globally complete (No seeds left or playbook finished)
      if (currentStrategyIdx >= TOTAL_STRATEGY_STEPS || robotInfo.seedsLeft <= 0) {
        if (!headingToExit) {
          //serial.println("Mission Complete or Out of Seeds! Locking Exit Coordinate.");
          headingToExit = true;
          robotInfo.finalDestination = {0, 7}; // Airlock exit coords
          newPathNeeded = true;
        }
      } 
      else if (newPathNeeded) {
        // Request a fresh occupancy map from the server before calculating a new path
        requestOccupancyMap(); 
        
        // Wait a brief moment for the MQTT callback to populate the map bytes
        unsigned long mapWait = millis();
        while (!occupancyMapIsReady() && millis() - mapWait < 300) {
          updateRobotCommunication();
          yield();
        }

        // Find the closest target matching our current playbook strategy rule
        while (currentStrategyIdx < TOTAL_STRATEGY_STEPS) {
          if (findNearestStrategicTarget()) {
            break; // Valid target found and loaded into robotInfo.finalDestination!
          }
          
          // If this playbook play yields absolutely no options left in the arena,
          // automatically skip to the next strategy rule!
//           Serial.print("[PLAN] Strategy step ");
//           Serial.print(currentStrategyIdx);
          //serial.println(" completely exhausted. Moving to next play...");
          currentStrategyIdx++;
        }

        // Double check if we fell off the end of the playbook while skipping empty strategies
        if (currentStrategyIdx >= TOTAL_STRATEGY_STEPS) {
          //serial.println("All strategy rules exhausted! Heading to exit.");
          headingToExit = true;
          robotInfo.finalDestination = {INTERNAL_AIRLOCK_B_X, INTERNAL_AIRLOCK_B_Y};
        }
      }

      // Check if current position is the destination
      if (robotInfo.currentPos.x == robotInfo.finalDestination.x && robotInfo.currentPos.y == robotInfo.finalDestination.y) {
        if (headingToExit) {
          //serial.println("Arrived at the Exit Airlock. Initiating exit Sequence...");
          float turnAngle = normaliseAngleDeg(AIRLOCK_HEADING - sensors.yaw);
          startTurnAngle(turnAngle);
          currentState = STATE_ALIGN_AIRLOCK_B;
        } else {
          //serial.println("Target reached! Deploying Planter...");
          startPlanting();
          currentState = STATE_PLANTING; 
        }
        break; 
      }

      // Path generation and execution pass-off
      if (newPathNeeded) {
        if (!calculateWeightedPath()) {
          //serial.println("ERROR: No valid path to target.");
          currentState = STATE_EMERGENCY_STOP;
          break;
        }
        robotInfo.currentWaypointIdx = 0;
        newPathNeeded = false;
      } else {
        robotInfo.currentWaypointIdx += 1;
      }

      Coordinate nextNode = robotInfo.getNextNode(); 
      float targetHeading = getTargetHeading(robotInfo.currentPos, nextNode);
      float turnAngle = normaliseAngleDeg(targetHeading - sensors.yaw);
      
      startTurnAngle(turnAngle); 
      currentState = STATE_TURN;
      break;
    }


    case STATE_STANDBY_BASE:
      stopMotors();
      break;

    case STATE_TASK2_BASE_EXIT: {
      int task2Result = updateTask2BaseExit();

      if (task2Result == 1) {
        //serial.println("SUCCESS: Task 2 complete. Cleared ramp and entered the arena!");

        requestFertilityCheck(sensors.rfidInfo);

        unsigned long apiWait = millis();
        while (!fertilityReplyReady() && millis() - apiWait < 500) {
          updateRobotCommunication();
          yield();
        }

        robotInfo.currentPos.x = INTERNAL_AIRLOCK_A_X;
        robotInfo.currentPos.y = INTERNAL_AIRLOCK_A_Y;

        snapYawToGrid();
        newPathNeeded = true;
        currentState = STATE_PLAN;
      } else if (task2Result < 0) {
        currentState = STATE_EMERGENCY_STOP;
      }
      break;
    }

      case STATE_CALIBRATING_IR:
      updateIRCalibration(); 
      
      if (isCalibrationComplete()) { 
        //serial.println("Robot Ready. Press the start switch to begin mission plan."); 
        stopMotors(); 

        startPlanWhenAllowed = true; 
        currentState = STATE_STANDBY_BASE; 
      }
      break;


    case STATE_NAVIGATING_LINES: {
      updateTask3LineNavigation(); 

      if (task3LineNavigationComplete()) {
        //serial.println("SUCCESS: Reached next node!");
        robotInfo.currentPos = robotInfo.getNextNode(); // Update Map Memory
        //YawToGrid();                                // EXPERIMENTAL round the gyro reading to snap to nearest 90 degrees while we are navigating grid lines
         
        currentState = STATE_PLAN; 

      } 
      else if (task3LineNavigationFailed()) {
         //serial.println("ERROR: Failsafe triggered (Lost line or timeout).");
//          Serial.print("Task 3 failure reason: ");
         //serial.println(getTask3LineNavigationFailureReason());
         currentState = STATE_EMERGENCY_STOP; 
         // currentState = STATE_PLAN;
      }
      break;
    }
    case STATE_NAVIGATING_OPEN: 
      updateTask4OpenNavigation();

      if (task4OpenNavigationComplete()) {
        //serial.println("SUCCESS: Reached next open-field node!");
        robotInfo.currentPos = robotInfo.getNextNode();
        snapYawToGrid();
        currentState = STATE_PLAN;
      }
      else if (task4OpenNavigationFailed()) {
        //serial.println("ERROR: Open-field navigation failed.");
        currentState = STATE_EMERGENCY_STOP;
      }
      
      break;

    
    case STATE_TURN:{
      Coordinate nextNode = robotInfo.getNextNode();

      if (updateTurnAngle()) {

        const int OBSTACLE_THRESHOLD_MM = 250;
        refreshAllSensors();

        if (sensors.tof_front < OBSTACLE_THRESHOLD_MM && sensors.tof_front > 0) {
//             Serial.print("OBSTACLE DETECTED at distance: ");
            //serial.println(sensors.tof_front);
            
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
    case STATE_PLANTING: {
      // Create a static step tracker to handle the multi-step network handshake
      static int plantingStep = 0; 
      static unsigned long apiTimeoutMs = 0;

      switch (plantingStep) {
        
        // 0: Scan RFID & Request Fertility Check ---
        case 0: {
          //serial.println("[PLANTER] Arrived at target node. Scanning RFID tag...");
          
          if (!rfidOnline) {
            //serial.println("[PLANTER] Warning: RFID reader offline! Forced to skip verification.");
            startPlanting(); // Fallback: just start planting blind
            plantingStep = 2; 
            break;
          }

          // Read the current RFID tag data
          refreshAllSensors(); 
          uint32_t currentTag = sensors.rfidInfo;

          if (currentTag == 0) {
            //serial.println("[PLANTER] Error: No RFID tag detected under the robot! Retrying scan...");
            // Optionally add a tiny timeout or just let it loop until a tag is found
            break;
          }

//           Serial.print("[PLANTER] Tag detected: "); //serial.println(currentTag, HEX);
          //serial.println("[PLANTER] Sending fertility check request to server...");
          
          requestFertilityCheck(currentTag);
          apiTimeoutMs = millis();
          plantingStep = 1; // Move to waiting for reply
          break;
        }

        // 1: Wait for Server Fertility Confirmation ---
        case 1: {
          // Check if the MQTT callback has updated our fertility status
          if (fertilityReplyReady()) {
            if (currentTagIsPlantable()) {
              //serial.println("[PLANTER] Server confirmed: TAG IS FERTILE. Deploying physical mechanism...");
              startPlanting(); // Trigger your hardware planting routine
              plantingStep = 2;
            } else {
              //serial.println("[PLANTER] Server rejected: Tag is already planted or infertile! Aborting.");
              plantingStep = 0; // Reset step for next time
              currentState = STATE_ADVANCE_AFTER_PLANTING; // Skip this spot safely
            }
          }
          // Network failsafe: if server doesn't reply in 1.5 seconds, plant anyway
          else if (millis() - apiTimeoutMs > 1500) {
            //serial.println("[PLANTER] API Timeout! Server didn't reply. Deploying anyway as failsafe...");
            startPlanting();
            plantingStep = 2;
          }
          break;
        }

        // 2: Physical Deployment 
        case 2: {
          if (updatePlanting()) {
            // Physical hardware actions completed successfully!
//             Serial.print("[PLANTER] Seed dropped. Seeds remaining: ");
            //serial.println(robotInfo.seedsLeft);

            // Report the successful plant back to the server to secure your points
            if (rfidOnline && sensors.rfidInfo != 0) {
              //serial.println("[PLANTER] Reporting seed deployment to server API...");
              reportSeedPlanted(sensors.rfidInfo);
            }

            // Move to the post-plant step and reset our step counter for the next run
            startPostPlantAdvance();
            plantingStep = 0; 
            currentState = STATE_ADVANCE_AFTER_PLANTING;
          }
          break;
        }
      }
      break;
    }
      
    case STATE_ADVANCE_AFTER_PLANTING:
      if (updatePostPlantAdvance()) {
        // Move to the next index in our playbook array {true, true, false, true, false}
        currentStrategyIdx++; 
//         Serial.print("[STRATEGY] Step complete. Advancing Playbook Index to: ");
        //serial.println(currentStrategyIdx);

        newPathNeeded = true;       // Force the planner to evaluate the new rule
        currentState = STATE_PLAN;  // Recalculate route to the next strategic target
      }
      break;

    case STATE_EMERGENCY_STOP:
      stopMotors();
      abortPlanting();
      break;
    case STATE_BASE_NAVIGATION:

    case STATE_RAMP: {
      if (updateRampWallFollowing(true) == 1) { 
        //serial.println("SUCCESS: Cleared the ramp and entered the arena!");
        
        // 1. Force a server request to pull the exact API coordinates of this Airlock
        requestFertilityCheck(sensors.rfidInfo);
        
        unsigned long apiWait = millis();
        while (!fertilityReplyReady() && millis() - apiWait < 500) {
          updateRobotCommunication();
          yield();
        }


        // Set our internal position safely
        robotInfo.currentPos.x = INTERNAL_AIRLOCK_A_X;
        robotInfo.currentPos.y = INTERNAL_AIRLOCK_A_Y;
        
//         Serial.print("[RAMP EXIT] Synced internal map to: (X:");
//         Serial.print(robotInfo.currentPos.x); Serial.print(", Y:");
//         Serial.print(robotInfo.currentPos.y); //serial.println(")");

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
          setRescueLedOverride(true);
          digitalWrite(RGB_GREEN_PIN, LOW); 
//           Serial.print("[RESCUE] Contact made! Illuminating GREEN LED and holding for ");
//           Serial.print(RESCUE_LED_HOLD_MS / 1000);
          //serial.println("s...");
        } 
        else if (rescueStatus == -1) {
          //serial.println("ABORT: Rescue timeout triggered.");
          currentState = STATE_EMERGENCY_STOP; 
        }
      } 
      

      else {

        stopMotors(); 
        
        if (millis() - contactTimeMs >= RESCUE_LED_HOLD_MS) {
          // Turn OFF Green LED 
          setRescueLedOverride(false);
          digitalWrite(RGB_GREEN_PIN, HIGH);
          digitalWrite(RGB_RED_PIN, HIGH);
          //digitalWrite(RGB_BLUE_PIN, HIGH);
          
//           Serial.print("[RESCUE] ");
//           Serial.print(RESCUE_LED_HOLD_MS / 1000);
          //serial.println("-second hold complete. Initiating return sequence.");
          
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
        //serial.println("Safely parked in the airlock!");
        currentState = STATE_AIRLOCK_B; 
      }
      break;

    case STATE_AIRLOCK_B: {
      static int exitStep = 0;
      static unsigned long exitTimerMs = 0;

      switch (exitStep) {
        
        // Scan the Airlock RFID Tag 
        case 0: {
          stopMotors();
          //serial.println("[EXIT] Waiting at Airlock B gate. Scanning RFID tag...");
          
          refreshAllSensors();
          uint32_t airlockTag = sensors.rfidInfo;

          if (airlockTag == 0) {
            //serial.println("[EXIT] Error: No RFID tag detected at the gate yet. Retrying scan...");
            delay(100); 
            break;
          }

//           Serial.print("[EXIT] Airlock Tag Detected: "); //serial.println(airlockTag, HEX);
          //serial.println("[EXIT] Requesting server to open Airlock B...");
          
          requestOpenAirlock('B', airlockTag);
          exitTimerMs = millis();
          exitStep = 1; 
          break;
        }

        // Wait for Server Approval 
        case 1: {
          if (airlockRequestAccepted()) {
            //serial.println("[EXIT] SERVER APPROVED! Gate opening. Preparing to move...");
            exitTimerMs = millis();
            exitStep = 2; 
          } 
          else if (airlockRequestRejected()) {
            //serial.println("[EXIT] Server REJECTED airlock request! Retrying in 2 seconds...");
            exitTimerMs = millis();
            exitStep = 4; 
          }
          else if (millis() - exitTimerMs > 5000) {
            //serial.println("[EXIT] Airlock API timeout. Resetting handshake...");
            exitStep = 0;
          }
          break;
        }

        // Clear the Gate Threshold Using PI Closed-Loop Control 
        case 2: {
          // Because walls are far away, this automatically drives straight 
          // forward at your exact target encoder speed with no steering jitter
          updateRampWallFollowing(true); 
          
          if (millis() - exitTimerMs > 1500) {
            //serial.println("[EXIT] Cleared the airlock zone. Actively scanning for home base line...");
            exitTimerMs = millis();
            exitStep = 3; 
          }
          break;
        }

        // Wall-Follow Down the Ramp & Scan for the Line 
        case 3: {
          // Walls close back up as you descend the ramp, enabling smooth tracking
          updateRampWallFollowing(true); 

          // Scan your IR array to check if you've crossed the flat ground marker line
          bool lineDetected = false;
          for (int i = 0; i < IR_COUNT; i++) {
            if (sensors.irLineArray[i] > 2000) { 
              lineDetected = true;
              break;
            }
          }

          if (lineDetected) {
            //serial.println("[EXIT] Line detected at base of ramp! Switching to home line tracking...");
            exitTimerMs = millis();
            exitStep = 5; 
          }
          break;
        }

        // Rejection Cooldown 
        case 4: {
          stopMotors();
          if (millis() - exitTimerMs > 2000) {
            exitStep = 0; 
          }
          break;
        }

        // Follow the Line Into the Base 
        case 5: {
          // Run your standard base line alignment routine here
          
          if (millis() - exitTimerMs > 2000) {
            //serial.println("=========================================");
            //serial.println("   MISSION ACCOMPLISHED: SAFELY AT BASE  ");
            //serial.println("=========================================");
            stopMotors();
            exitStep = 6; 
          }
          break;
        }

        // --- STEP 6: Final Shutdown ---
        case 6:
          stopMotors();
          break;
      }
      break;
    }
    case STATE_DEBUG: {
      // 1. Keep motors powered down so you can spin the wheels safely by hand
      stopMotors();

      // 2. Throttled print timer to protect the Mbed OS WiFi thread
      static unsigned long lastEncoderPrintMs = 0;
      if (millis() - lastEncoderPrintMs >= 200) { // Updates 5 times a second
        lastEncoderPrintMs = millis();

        //serial.println("\n===== ENCODER LIVE TRACKING =====");
        
//         Serial.print("Left Encoder Position  (Pin D38): ");
        //serial.println(leftEncoderPos);
        
//         Serial.print("Right Encoder Position (Pin D39): ");
        //serial.println(rightEncoderPos);
        
        //serial.println("=================================");
        //serial.println("[Tip] Spin wheels forward -> counts should INCREASE.");
        //serial.println("[Tip] Spin wheels backward -> counts should DECREASE.");
      }
      break;
    }
  }
}
