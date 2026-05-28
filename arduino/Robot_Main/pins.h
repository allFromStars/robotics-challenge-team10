#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

// ============================================================
// Motoron motor channels
// ============================================================

const int LEFT_MOTOR_CHANNEL = 1;
const int RIGHT_MOTOR_CHANNEL = 2;

// Motor direction correction.
// Keep these consistent with the tested Robot_Main / robot_demo code.
const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;
const int FORWARD_SIGN = -1;

// ============================================================
// Encoder pins
// ============================================================

const int LEFT_ENCODER_PIN_A = D2;
const int LEFT_ENCODER_PIN_B = D3;

const int RIGHT_ENCODER_PIN_A = D4;
const int RIGHT_ENCODER_PIN_B = D5;

// ============================================================
// IR reflectance sensor array
// ============================================================

const uint8_t IR_COUNT = 9;

const uint8_t IR_PINS[IR_COUNT] = {
  22, 23, 24, 25, 26, 27, 28, 29, 30
};

const uint16_t IR_TIMEOUT_MICROS = 2500;

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
// front  -> Serial1
// left   -> Serial2
// right1 -> Serial3
// right2 -> Serial4
//
// These are not pin constants because HardwareSerial objects are used
// directly in the code.

const unsigned long TOF_SENSOR_BAUD = 921600;

// ============================================================
// Button / LED pins
// ============================================================

// From robot_demo.
const int BUTTON_PIN = D8;

// RGB LED pins from robot_demo.
// Current code assumes common cathode.
const int RGB_RED_PIN = D52;
const int RGB_GREEN_PIN = D50;

// Add blue pin here if installed later.
// const int RGB_BLUE_PIN = ...;

#endif