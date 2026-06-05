#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

// Servo motor
const int SERVO_PIN = A1;

// ============================================================
// Motoron motor channels
// ============================================================

const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;

// Motor direction correction.

// ============================================================
// Encoder pins
// ============================================================

/*
const int LEFT_ENCODER_PIN_A = D2;
const int LEFT_ENCODER_PIN_B = D3;

const int RIGHT_ENCODER_PIN_A = D4;
const int RIGHT_ENCODER_PIN_B = D5;
*/

//NEW TO SAVE ARDUINO FROM INTERRUPT AND WIFI PROBLEMS
const int LEFT_ENCODER_PIN_A  = D38; // PI_14 (EXTI Line 14) - Safe Active Interrupt!
const int LEFT_ENCODER_PIN_B  = D36; // Standard Digital Read

const int RIGHT_ENCODER_PIN_A = D39; // PE_6  (EXTI Line 6)  - Safe Active Interrupt!
const int RIGHT_ENCODER_PIN_B = D37; // Standard Digital Read


// ============================================================
// IR reflectance sensor array
// ============================================================

const uint8_t IR_COUNT = 9; 

const uint8_t irPins[IR_COUNT] = {
  22, 23, 24, 25, 26, 27 , 28, 29, 30
};

// ============================================================
// RFID reader
// ============================================================

// RFID reader is connected on Wire1 I2C bus.
const uint8_t RFID_ADDR = 0x28;

// ============================================================
// IMU
// ============================================================

// ICM-20948 is connected on Wire I2C bus.
// Default address used in current code is 0x68.
const uint8_t IMU_ADDR = 0x68;

// ============================================================
// TOF distance sensors
// ============================================================

// Current Robot_Main uses hardware serial ports:
//
// front  -> Serial1 -> D0 / D1
// left   -> Serial2 -> D19 / D18
// right1 -> Serial3 -> D17 / D16
// right2 -> Serial4 -> D15 / D14
//
// These are not pin constants because HardwareSerial objects are used
// directly in the code.

// ============================================================
// Button / LED pins
// ============================================================

// From robot_demo.
const int BUTTON_PIN = D8;

// RGB LED pins from robot_demo.
// Current code assumes common anode: HIGH = off, LOW = on.
const int RGB_RED_PIN = D52;
const int RGB_GREEN_PIN = D53;

// Add blue pin here if installed later.
// const int RGB_BLUE_PIN = ...;

const int BUMPER_PIN = D50;


#endif
