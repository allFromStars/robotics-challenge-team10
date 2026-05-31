#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// Motor configuration
// ============================================================

const int MAX_ACCEL = 150;
const int MAX_DECEL = 250;

const int LEFT_DIR = 1;
const int RIGHT_DIR = -1;
const int FORWARD_SIGN = 1;

const int MOTOR_COMMAND_TIMEOUT_MS = 1000;

const int MaxMotorSpeed = 600;

// ============================================================
// --- TURN PID TUNING ---
// ============================================================
const float turnKp = 15.5;
const float turnKi = 1.8;
const float turnKd = 1.8;

const int MOTOR_MAX_LIMIT = 500;  
const int MOTOR_MIN_LIMIT = 200;


// ============================================================
// TOF distance sensor configuration
// ============================================================

const unsigned long SENSOR_BAUD = 921600;
const int TOF_FILTER_SIZE = 10;

const int32_t TOF_FRONT_OFFSET = 0;
const int32_t TOF_LEFT_OFFSET = 0;
const int32_t TOF_RIGHT1_OFFSET = 0;
const int32_t TOF_RIGHT2_OFFSET = 0;

// ============================================================
// IR reflectance sensor configuration
// ============================================================

const uint16_t IR_TIMEOUT_MICROS = 2500;

// ============================================================
// IMU configuration
// ============================================================

const float GYRO_DEADBAND_DPS = 0.1;
const float MAG_DECLINATION_DEG = 4.0;

// ============================================================
// Robot default setup
// ============================================================

const int STARTING_SEED_COUNT = 5;

const int START_X = -1;
const int START_Y = -1;

const int DEFAULT_DESTINATION_X = 5;
const int DEFAULT_DESTINATION_Y = 5;

// ============================================================
// Debug / timing
// ============================================================

const unsigned long DEBUG_PRINT_INTERVAL_MS = 250;

#endif