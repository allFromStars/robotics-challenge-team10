#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

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
  STATE_STANDBY_BASE,       // Parked at base waiting for start cue
  STATE_BASE_NAVIGATION,

  STATE_RAMP,

  STATE_PLAN, 

  STATE_TURN,         // Go back to plan
  STATE_ALIGNCOMPASS,

  STATE_NAVIGATING_LINES,     // Tracking line in arena
  STATE_NAVIGATING_OPEN,      // Navigating in open field

  STATE_ALIGN_SEED,         // After wanted RFID detected, creep forward and use reflectance to align itself slowly
  STATE_PLANTING,           // Paused over an RFID seed hole, activating the hopper mechanism

  STATE_RESCUE_MODE,        // received signal to save robot that is right infront, tap robot
  STATE_STRANDED_ALIVE,     // Stop wheels activate LED, do stranded protocol

  STATE_REVIVED_RETURN,     // Revived - do revival protocol
  STATE_EMERGENCY_STOP,      // Hardware kill switch fallback state

  STATE_EXIT_ARENA,

  STATE_AIRLOCK_B,

  STATE_DEBUG,
};
#endif