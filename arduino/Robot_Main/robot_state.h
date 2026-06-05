#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

const int GRID_WIDTH  = 9; 
const int GRID_HEIGHT = 9;
bool newPathNeeded = true;

// 1 = Grid Line Area (.) -> Navigation Weight: 1
// 2 = Open Field Area (-) -> Navigation Weight: 2 * Multiplier
// 3 = Solid Obstacle  (X) -> Impassable Wall

byte arenaMap[GRID_HEIGHT][GRID_WIDTH] = {
  {1, 1, 1, 1, 1, 2, 2, 2, 2}, // Y = 8 (Top Row of Grid)
  {1, 1, 1, 1, 1, 2, 2, 2, 2}, // Y = 7
  {1, 1, 1, 1, 1, 1, 2, 2, 2}, // Y = 6 (Horizontal Obstacle Wall)
  {1, 1, 1, 1, 1, 2, 2, 2, 2}, // Y = 5
  {1, 1, 1, 1, 1, 2, 2, 2, 2}, // Y = 4
  {1, 1, 1, 1, 2, 2, 2, 2, 2}, // Y = 3 (Vertical Obstacle Block)
  {1, 1, 1, 1, 2, 2, 2, 2, 2}, // Y = 2 (Vertical Obstacle Block)
  {1, 1, 1, 1, 2, 2, 2, 2, 2}, // Y = 1
  {1, 1, 1, 1, 2, 2, 2, 2, 2}  // Y = 0 (Bottom Row of Grid)
};

// STANDARD MATH TRACKING VECTORS
// Index 0: Move NORTH ↑ (X stays same, Y increases by 1)
// Index 1: Move EAST  → (X increases by 1, Y stays same)
// Index 2: Move SOUTH ↓ (X stays same, Y decreases by 1)
// Index 3: Move WEST  ← (X decreases by 1, Y stays same)
const int dx[] = {0, 1, 0, -1};
const int dy[] = {1, 0, -1, 0};

/* enum RobotState {
  STATE_STANDBY_BASE,       // Parked at base waiting for start cue
  STATE_LINE_FOLLOWING_B,     // Tracking line in base
  STATE_PLAN, 
  STATE_NAVIGATING_LINES,     // Tracking line in arena
  STATE_NAVIGATING_OPEN,      // Navigating in open field
  STATE_WALL_FOLLOWING,     // Wall following in Arena
  STATE_ALIGN_SEED,         // After wanted RFID detected, creep forward and use reflectance to align itself slowly
  STATE_PLANTING_CYCLE,     // Paused over an RFID seed hole, activating the hopper mechanism
  STATE_RESCUE_MODE,        // received signal to save robot that is right infront, tap robot
  STATE_STRANDED_ALIVE,     // Stop wheels activate LED, do stranded protocol
  STATE_REVIVED_RETURN,     // Revived - do revival protocol
  STATE_EMERGENCY_STOP      // Hardware kill switch fallback state
}; */

enum RobotState {
  STATE_CALIBRATING_IR,
  STATE_STANDBY_BASE,       // Parked at base waiting for start cue
  STATE_BASE_NAVIGATION,
  STATE_TASK2_BASE_EXIT,

  STATE_RAMP,

  STATE_PLAN, 

  STATE_TURN,         // Go back to plan
  STATE_ALIGNCOMPASS,

  STATE_NAVIGATING_LINES,     // Tracking line in arena
  STATE_NAVIGATING_OPEN,      // Navigating in open field

  STATE_ALIGN_SEED,         // After wanted RFID detected, creep forward and use reflectance to align itself slowly
  STATE_PLANTING,           // Paused over an RFID seed hole, activating the hopper mechanism
  STATE_ADVANCE_AFTER_PLANTING, // Move from planting pose to turn-center pose

  STATE_RESCUE_MODE,        // received signal to save robot that is right infront, tap robot
  STATE_STRANDED_ALIVE,     // Stop wheels activate LED, do stranded protocol

  STATE_REVIVED_RETURN,     // Revived - do revival protocol
  STATE_EMERGENCY_STOP,      // Hardware kill switch fallback state

  STATE_EXIT_ARENA,

  STATE_ALIGN_AIRLOCK_B,     //rotate toward airlock
  STATE_AIRLOCK_B,

  STATE_DEBUG,
};

enum LineArrivalMode {
  LINE_ARRIVE_FOR_TURN,
  LINE_ARRIVE_FOR_PLANTING
};

#endif
