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

const int MOTOR_MAX_LIMIT = 300;  
const int MOTOR_MIN_LIMIT = 150;
const int MOTOR_TURN_FINISH_LIMIT = 140;
const int MOTOR_TURN_STUCK_LIMIT = 180;
const float TURN_INTEGRAL_ACTIVE_ERROR_DEG = 20.0;
const float TURN_SLOWDOWN_ERROR_DEG = 12.0;
const unsigned long TURN_DEBUG_PRINT_INTERVAL_MS = 100;
const unsigned long TURN_STUCK_CHECK_INTERVAL_MS = 600;
const float TURN_STUCK_MIN_PROGRESS_DEG = 1.0;


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
// Line navigation configuration
// ============================================================

const int GRID_LINE_SPEED = 280;
const int AFTER_RFID_SLOW_SPEED = 220;
const int REACQUIRE_SPEED = 220;
const int LINE_NAV_MAX_SPEED = 360;

const float lineKp = 0.04;
const float lineKd = 1;
const float headingKp = 4.0;
const int HEADING_MAX_CORRECTION = 70;

const long MIN_COUNTS_BEFORE_NODE_CANDIDATE = 250;
const unsigned long NODE_CANDIDATE_CONFIRM_MS = 80;
const unsigned long RFID_TO_HOLE_TIMEOUT_MS = 1800;
const unsigned long NODE_PAUSE_MS = 600;
const int TURN_CENTER_ADVANCE_SPEED = 180;
const long TURN_CENTER_ADVANCE_COUNTS = 900;
const unsigned long TURN_CENTER_ADVANCE_TIMEOUT_MS = 1500;
const int POST_PLANT_ADVANCE_SPEED = 180;
const long POST_PLANT_ADVANCE_COUNTS = 900;
const unsigned long POST_PLANT_ADVANCE_TIMEOUT_MS = 1500;
const unsigned long START_NODE_EXIT_TIMEOUT_MS = 2500;
const unsigned long REACQUIRE_AFTER_NODE_TIMEOUT_MS = 1800;
const unsigned long LINE_NAV_TIMEOUT_MS = 45000;

const int LINE_CENTER = 4000;
const uint16_t LINE_THRESHOLD = 400;
const uint16_t LINE_TOTAL_THRESHOLD = 300;

// ============================================================
// Open-field one-edge navigation configuration
// ============================================================

const int OPEN_FIELD_SPEED = 320;
const int OPEN_FIELD_SLOW_SPEED = 280;
const int OPEN_SEARCH_SPEED = 280;
const int OPEN_NAV_MAX_SPEED = 360;

// Measured encoder count for one node-to-node edge.
const long OPEN_GRID_EDGE_COUNTS = 4500;

// Start searching before the estimated node position.
const long OPEN_NODE_SEARCH_WINDOW_COUNTS = 900;

const float OPEN_HEADING_KP = 4.0;
const int OPEN_HEADING_MAX_CORRECTION = 80;

const unsigned long OPEN_NODE_PAUSE_MS = 700;
const unsigned long OPEN_EDGE_TIMEOUT_MS = 10000;
const unsigned long OPEN_NODE_SEARCH_TIMEOUT_MS = 5000;

// ============================================================
// IMU configuration
// ============================================================

const float GYRO_DEADBAND_DPS = 0.8;
const float MAG_DECLINATION_DEG = 4.0;

// ============================================================
// Robot default setup
// ============================================================

const int STARTING_SEED_COUNT = 5;

const int START_X = 0;
const int START_Y = 0;

const int DEFAULT_DESTINATION_X = 5;
const int DEFAULT_DESTINATION_Y = 5;

// ============================================================
// Debug / timing
// ============================================================

const unsigned long DEBUG_PRINT_INTERVAL_MS = 250;

#endif
