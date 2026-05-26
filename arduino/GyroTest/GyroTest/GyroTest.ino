#include <Wire.h>
#include "ICM_20948.h"
#include <Motoron.h>



ICM_20948_I2C myICM;
MotoronI2C mc(16);

const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;
const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;
const int FORWARD_SIGN = -1; 

float yaw = 0;
unsigned long lastTimeMicros;
float gyroBiasZ = 0;

float turnKp = 15.5;
float turnKd = 1.8;   

const int MOTOR_MAX_LIMIT = 300;
const int MOTOR_MIN_LIMIT = 100;  // Minimum power required to overcome floor

void driveMotors(int leftLogicalSpeed, int rightLogicalSpeed) {
  mc.setSpeed(LEFT_MOTOR_CHANNEL, LEFT_DIR * leftLogicalSpeed);
  mc.setSpeed(RIGHT_MOTOR_CHANNEL, RIGHT_DIR * rightLogicalSpeed);
}

void stopMotors() {
  mc.setSpeed(LEFT_MOTOR_CHANNEL, 0);
  mc.setSpeed(RIGHT_MOTOR_CHANNEL, 0);
}

// Call this function with a relative angle target (e.g., turnAngle(90.0) or turnAngle(-90.0))
void turnAngle(float targetAngle) {
  Serial.print("Initiating Pivot Turn to: "); Serial.print(targetAngle); Serial.println("°");
  

  float targetYaw = yaw + targetAngle; 
  float error = targetAngle;
  float lastTurnError = error;
  float turnIntegral = 0;
  
  unsigned long lastLoopMicros = micros();
  unsigned long settleStartTime = 0;
  bool turnComplete = false;

  while (!turnComplete) {

    mc.resetCommandTimeout();

    if (myICM.dataReady()) {
      myICM.getAGMT();


      unsigned long currentMicros = micros();
      float dt = (currentMicros - lastLoopMicros) / 1000000.0;
      lastLoopMicros = currentMicros;


      float gyroZ = myICM.gyrZ() - gyroBiasZ;
      if (abs(gyroZ) > 0.1) {
        yaw += gyroZ * dt;
      }

      error = targetYaw - yaw;
      

      float P_out = error * turnKp;


      turnIntegral += error * dt;
      if (turnIntegral > 1000)  turnIntegral = 1000;
      if (turnIntegral < -1000) turnIntegral = -1000;
      float I_out = turnIntegral * turnKi;


      float D_out = ((error - lastTurnError) / dt) * turnKd;
      lastTurnError = error;

      float totalOutput = P_out + I_out + D_out;

      int leftSpeed = 0;
      int rightSpeed = 0;

 
      if (abs(totalOutput) < MOTOR_MIN_LIMIT && abs(error) > 0.8) {
        totalOutput = (totalOutput > 0) ? MOTOR_MIN_LIMIT : -MOTOR_MIN_LIMIT;
      }


      if (totalOutput > MOTOR_MAX_LIMIT)  totalOutput = MOTOR_MAX_LIMIT;
      if (totalOutput < -MOTOR_MAX_LIMIT) totalOutput = -MOTOR_MAX_LIMIT;


      leftSpeed  = FORWARD_SIGN * totalOutput; 
      rightSpeed = -FORWARD_SIGN * totalOutput;

      driveMotors(leftSpeed, rightSpeed);


      if (abs(error) <= 1.0) {
        if (settleStartTime == 0) {
          settleStartTime = millis();
        }
        if (millis() - settleStartTime >= 80) {
          turnComplete = true;
        }
      } else {
        settleStartTime = 0; 
      }
      
      // Fast diagnostic readout
      Serial.print("Target: "); Serial.print(targetYaw, 1);
      Serial.print(" | Current: "); Serial.print(yaw, 1);
      Serial.print(" | Error: "); Serial.println(error, 1);
    }
  }

  stopMotors();
  Serial.println("Target Secured! Resuming main operations.");
  delay(100); 
}

// =============================================================================
// SYSTEM SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500); 
  Serial.println("System Starting...");


  Wire.begin();  
  Wire1.begin();
  Wire.setClock(400000);
  delay(100);


  mc.setBus(&Wire1);
  mc.reinitialize();
  mc.clearResetFlag();
  mc.setCommandTimeoutMilliseconds(1000);
  mc.setMaxAcceleration(LEFT_MOTOR_CHANNEL, 120);
  mc.setMaxDeceleration(LEFT_MOTOR_CHANNEL, 250);
  mc.setMaxAcceleration(RIGHT_MOTOR_CHANNEL, 120);
  mc.setMaxDeceleration(RIGHT_MOTOR_CHANNEL, 250);
  stopMotors();

  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, 0); // Using 0 for 0x68
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.print("Sensor connection failed: ");
      Serial.println(myICM.statusString());
      delay(500);
    } else {
      initialized = true;
    }
  }

  Serial.println("ICM20948 Connected! Calibrating...");
  
//calibration
  int samples = 0;
  while(samples < 1000) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      gyroBiasZ += myICM.gyrZ();
      samples++;
      if(samples % 50 == 0) Serial.print("."); 
    }
    delay(5);
  }
  gyroBiasZ /= 1000.0;
  
  Serial.println("\nCalibration Done! Ready to Navigate.");
  lastTimeMicros = micros();
  
  // TEST
  
  delay(2000);
  turnAngle(90.0); 
  
  delay(2000);
  turnAngle(-90.0); //
  
}


void loop() {
  mc.resetCommandTimeout();


  if (myICM.dataReady()) {
    myICM.getAGMT();

    unsigned long currentTimeMicros = micros();
    float dt = (currentTimeMicros - lastTimeMicros) / 1000000.0;
    lastTimeMicros = currentTimeMicros;

    float gyroZ = myICM.gyrZ() - gyroBiasZ;

    if (abs(gyroZ) > 0.1) { 
      yaw += gyroZ * dt;
    }


    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 250) {
      lastPrint = millis();
      Serial.print("Current Global Yaw: ");
      Serial.println(yaw);
    }
  }
}