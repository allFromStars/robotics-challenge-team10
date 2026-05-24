#include <Wire.h>
#include <Arduino_BMI270.h> 
//#include <MadgwickAHRS.h>   //perhaps good library for IMU
#include <MFRC522_I2C.h>

// --- RFID Setup ---
#define RFID_ADDR 0x28
MFRC522_I2C rfid(RFID_ADDR, -1, &Wire1);
bool rfidCardPresent = false;

// --- TOF Distance Sensor Setup ---
const unsigned long SENSOR_BAUD = 921600;
const int TOF_FILTER_SIZE = 10;
int32_t offset1 = 0, offset2 = 0, offset3 = 0, offset4 = 0; // Calibration offsets

// --- IR Reflectance Array Setup ---
const uint8_t IR_COUNT = 9;
const uint8_t irPins[IR_COUNT] = { 22, 23, 24, 25, 26, 27, 28, 29, 30 };
const uint16_t IR_TIMEOUT_MICROS = 2500;


// struct of all sensor readings
struct SENSORS {
  float yaw;
  uint32_t tof_front;
  uint32_t tof_left;
  uint32_t tof_right1;
  uint32_t tof_right2;
  uint16_t irLineArray[IR_COUNT];
  bool rfidDetected;
}; 
SENSORS sensors;



void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } // Wait for PC Serial Monitor connection
  delay(1000);
  
  Serial.println("Initialising Sensors...");

  Wire1.begin();
  delay(100);

  // Initialise RFID reader
  rfid.PCD_Init();
  Serial.println("RFID Reader Initialised.");

  // Initialise ports for distance sensors
  Serial1.begin(SENSOR_BAUD);
  Serial2.begin(SENSOR_BAUD);
  Serial3.begin(SENSOR_BAUD);
  Serial4.begin(SENSOR_BAUD);
  Serial.println("[OK] Hardware UART Serial1-4 Open at 921600 Baud.");

  // Initialise Reflectance sensor pins
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(irPins[i], INPUT);
  }
  Serial.println("9-Pin IR Reflectance Array Pins Set.");
  
  Serial.println("Complete! \n");
  lastIMUUpdateMicros = micros();
}

void loop() {

  refreshAllSensors();

  // State machine here


  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 250) {
    lastPrint = millis();
    
    Serial.print(" | TOF Distances: ");
    Serial.print(sensors.tof_front); Serial.print(", ");
    Serial.print(sensors.tof_left); Serial.print(", ");
    Serial.print(sensors.tof_right1); Serial.print(", ");
    Serial.print(sensors.tof_right2);
    Serial.print(" | IR Center (S5): "); Serial.println(irValues[4]);
  }
}