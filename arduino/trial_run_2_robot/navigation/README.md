# Navigation Sketches

This folder contains final-style navigation prototypes for the arena tasks.
They are standalone Arduino sketches for testing, but their internal structure is designed so the behaviour can later be moved into `Robot_Main`.

## Folder Structure

```text
navigation/
  task3_line_rfid_node_navigation/
    task3_line_rfid_node_navigation.ino

  task4_open_hole_navigation/
    task4_open_hole_navigation.ino
```

## Task 3: Solid Grid Navigation

File:

```text
task3_line_rfid_node_navigation/task3_line_rfid_node_navigation.ino
```

Scenario:

The robot is on the left half of the main arena, where the node holes are connected by solid black grid lines.

Strategy:

```text
follow the grid line
-> RFID reader detects the node tag first
-> slow down and keep following the line
-> IR array reaches the hole and briefly loses the line
-> confirm the node
-> pause briefly
-> use IMU heading hold to leave the hole
-> reacquire the black line
```

Sensors used:

```text
IR reflectance array: line following
RFID: early node/tag detection
IMU: heading hold while leaving the hole
Encoders: minimum-distance guard against immediate false node detection
```

Main controller:

```text
IR PD line following + RFID-before-hole node detection + IMU reacquire
```

Main states:

```text
T3_LINE_IDLE
T3_LINE_FOLLOW
T3_NODE_CANDIDATE
T3_NODE_CONFIRMED_PAUSE
T3_REACQUIRE_AFTER_NODE
T3_LINE_DONE
T3_LINE_FAILSAFE
```

Important parameters:

```cpp
const int TARGET_CONFIRMED_NODES = 4;
const int GRID_LINE_SPEED = 200;
const int AFTER_RFID_SLOW_SPEED = 120;
const int REACQUIRE_SPEED = 95;
const long MIN_COUNTS_BEFORE_NODE_CANDIDATE = 250;
const unsigned long RFID_TO_HOLE_TIMEOUT_MS = 1800;
const unsigned long NODE_PAUSE_MS = 600;
const unsigned long REACQUIRE_AFTER_NODE_TIMEOUT_MS = 1800;
```

Because the RFID reader is physically ahead of the IR array, Task 3 treats RFID as an early warning. The robot only confirms the node when the IR array later reaches the hole.

## Task 4: Open-Field Dead Reckoning

File:

```text
task4_open_hole_navigation/task4_open_hole_navigation.ino
```

Scenario:

The robot is on the right half of the main arena, where there are node holes but no solid black grid lines.

Strategy:

```text
drive toward the next open-field node using encoder distance and IMU heading hold
-> when near the expected node distance, slow down and search
-> use IR hole detection to find the node hole
-> align the hole near the center of the IR array
-> optionally use RFID to confirm the node identity
-> pause briefly
-> reset encoder count
-> drive the next segment
```

Sensors used:

```text
IMU: heading hold
Encoders: distance from one node to the next
IR reflectance array: wood-baseline hole detection and hole centering
RFID: optional node identity confirmation
```

Main controller:

```text
IMU P heading hold + encoder near-node estimate + IR hole detection/alignment
```

Main states:

```text
T4_OPEN_IDLE
T4_OPEN_DRIVE_SEGMENT
T4_OPEN_SEARCH_NODE
T4_OPEN_ALIGN_HOLE
T4_OPEN_CONFIRM_NODE
T4_OPEN_NODE_PAUSE
T4_OPEN_DONE
T4_OPEN_FAILSAFE
```

Important parameters:

```cpp
const long OPEN_GRID_SEGMENT_COUNTS = 1200;
const int OPEN_SEGMENTS_TO_RUN = 3;
const int OPEN_FIELD_SPEED = 140;
const int OPEN_FIELD_SLOW_SPEED = 90;
const int OPEN_SEARCH_SPEED = 80;
const int OPEN_ALIGN_SPEED = 90;
const long OPEN_NODE_SEARCH_WINDOW_COUNTS = 350;
const float OPEN_HEADING_KP = 4.0;
const int OPEN_HEADING_MAX_CORRECTION = 80;
const bool REQUIRE_RFID_CONFIRMATION = false;
const uint8_t HOLE_EMPTY_MIN_COUNT = 2;
const float HOLE_CENTER_TOLERANCE = 0.35;
const unsigned long OPEN_NODE_PAUSE_MS = 700;
```

`OPEN_GRID_SEGMENT_COUNTS` is still important, but it is no longer the final proof that a node has been reached. It tells the robot when to start looking for the hole. The hole is then detected using IR values relative to the calibrated wood baseline.

## Relationship Between Task 3 and Task 4

Both behaviours use the same high-level idea:

```text
move one segment
-> detect/confirm node
-> pause at node
-> continue to next segment
```

The difference is the movement controller:

```text
Task 3:
  IR line following keeps the robot on the solid grid line.

Task 4:
  IMU heading hold keeps the robot travelling straight without a line.
  IR hole detection confirms and aligns the node near the expected distance.
```

This means both can later be controlled by `STATE_PLAN`:

```text
STATE_PLAN
  -> choose next target
  -> choose line navigation or open-field navigation
  -> configure segment count / heading
  -> switch to navigation state
```

## Intended Final Integration

The final `Robot_Main` state machine can use these behaviours like this:

```cpp
case STATE_PLAN:
  if (nextMoveUsesLine) {
    startTask3LineNavigation();
    currentState = STATE_NAVIGATING_LINES;
  } else {
    startTask4OpenNavigation();
    currentState = STATE_NAVIGATING_OPEN;
  }
  break;

case STATE_NAVIGATING_LINES:
  updateTask3LineNavigation();
  if (task3LineNavigationComplete()) currentState = STATE_PLAN;
  if (task3LineNavigationFailed()) currentState = STATE_EMERGENCY_STOP;
  break;

case STATE_NAVIGATING_OPEN:
  updateTask4OpenNavigation();
  if (task4OpenNavigationComplete()) currentState = STATE_PLAN;
  if (task4OpenNavigationFailed()) currentState = STATE_EMERGENCY_STOP;
  break;
```

For final integration, the fixed constants:

```cpp
GRID_SEGMENTS_TO_RUN
OPEN_SEGMENTS_TO_RUN
targetHeadingDeg
```

should become runtime values supplied by the planner. In the current sketches, these are standalone-test constants.

## Testing Order

Recommended order:

```text
1. Measure encoder counts for one grid segment.
2. Tune Task 3 line following, RFID pending timing, and line reacquire.
3. Tune Task 4 IMU heading hold on a straight open-field run.
4. Calibrate Task 4 wood baseline and tune hole detection thresholds.
5. Tune Task 4 near-node search and hole alignment.
6. Verify RFID confirmation once IR hole detection is stable.
7. Connect both behaviours to STATE_PLAN.
```

## Notes

The standalone sketches still use blocking setup/calibration steps. This is acceptable for isolated testing.
During movement, the update functions are non-blocking and should be called once per loop.
