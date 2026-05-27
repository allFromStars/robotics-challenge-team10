// Semicolon and assignment bugs inside initWire() fixed
void initWire() {
  Serial.println("Initialising wire...");
  Wire.begin();
  Wire.setClock(400000);
  Wire1.begin();
  Wire1.setClock(400000); 
  delay(100); 
}

bool initMotors() {
  if (!checkI2CDevice(Wire1, 16)) {
    return false;
  }

  mc.setBus(&Wire1);
  mc.reinitialize();
  mc.clearResetFlag();
  mc.setCommandTimeoutMilliseconds(1000);
  
  mc.setMaxAcceleration(LEFT_MOTOR_CHANNEL, maxAccel);
  mc.setMaxDeceleration(LEFT_MOTOR_CHANNEL, maxDecel);
  mc.setMaxAcceleration(RIGHT_MOTOR_CHANNEL, maxAccel);
  mc.setMaxDeceleration(RIGHT_MOTOR_CHANNEL, maxDecel);
  
  stopMotors();
  return true;
}

void driveMotors(int leftLogicalSpeed, int rightLogicalSpeed) {
  if (motorOnline) {
    mc.setSpeed(LEFT_MOTOR_CHANNEL, LEFT_DIR * leftLogicalSpeed);
    mc.setSpeed(RIGHT_MOTOR_CHANNEL, RIGHT_DIR * rightLogicalSpeed);
  }
}

void stopMotors() {
  if (motorOnline) {
    mc.setSpeed(LEFT_MOTOR_CHANNEL, 0);
    mc.setSpeed(RIGHT_MOTOR_CHANNEL, 0);
  }
}