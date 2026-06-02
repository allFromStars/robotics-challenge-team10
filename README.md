# UCL Year 1 Robotics Challenge | Team 10

[![UCL Engineering](https://img.shields.io/badge/UCL-Engineering-lightgrey?style=flat-square&color=blue)](https://www.ucl.ac.uk/)
[![Framework](https://img.shields.io/badge/Framework-Arduino%20%2F%20C%2B%2B-orange?style=flat-square)](https://www.arduino.cc/)
[![Architecture](https://img.shields.io/badge/Architecture-Non--Blocking%20State%20Machine-brightgreen?style=flat-square)]()

An advanced, event-driven autonomous mobile robot platform developed for the Term 3 UCL Robotics Engineering Challenge. 

## The Team
* **Kaname Asaki**
* **Yunhe Xia**
* **Zeina Hosni**
* **Maahi Patil**

---

## Core Software Architecture

The platform utilizes a customized **Event-Driven Non-Blocking State Machine** running within a high-frequency polling loop. Unlike rigid sequence-driven systems, this architecture allows the robot to continuously refresh its sensor array and listen for safety overrides simultaneously while computing real-time control adjustments.

### Master State Machine




---

## Task-by-Task Control Flowcharts

### Task 1: Standard Line Tracking
Utilizes a high-speed Infrared (IR) reflective array paired with a **Proportional-Derivative (PD) controller** to calculate real-time differential wheel adjustments, keeping the robot centered on guidelines.

<img width="500" alt="Task 1 Flowchart" src="https://github.com/user-attachments/assets/e59b0f3e-4e04-47a7-a80f-d33a01cc76af" />

### Task 2: Intersection and Tag Alignment
Implements an event-driven intersection parsing system. When an intersection threshold is crossed, the robot scans for localized underbelly RFID tags to translate raw data into true Cartesian grid nodes.

<img width="500" alt="Task 2 Flowchart" src="https://github.com/user-attachments/assets/ab8e2864-f93a-4c1c-914e-3edb3ba94e04" />

### Task 3: Solid Grid Navigation
Combines a global pathfinding layer with localized line-following execution primitives to evaluate the map and navigate around arena grid layouts safely.

<img width="500" alt="Task 3 Flowchart" src="https://github.com/user-attachments/assets/a0beeca7-9684-4537-9cd6-323a5f174b5b" />

### Task 4: Open Field Dead-Reckoning
Executes precise maneuvers across track regions lacking explicit physical line guides, leveraging wheel encoder tracking and IMU yaw stability logic to minimize cumulative drift.

<img width="500" alt="Task 4 Flowchart" src="https://github.com/user-attachments/assets/dee4b6a6-6ce4-47d0-ae34-2b3cdadd00b2" />

### Task 5: Ramped Incline/Decline Control
A dual-layer **Adaptive Cruise Control (ACC) and Proportional-Integral (PI) Velocity Controller** designed to handle steep grades. Uses active reverse-PWM engine braking downhill and dynamic torque compensation uphill while monitoring obstacle distances ahead.

<img width="500" alt="Task 5 Flowchart" src="https://github.com/user-attachments/assets/acffa5ba-ceba-4e3b-ba33-22e17de81b09" />

### Task 8: Touch-Based Revival
A hardware-vetted rescue routine. It uses front Time-of-Flight (ToF) metrics to apply a smooth proportional deceleration curve, moving to a low-speed crawl until a physical limit switch triggers an instant motor stop, locking the state and flaring an RGB confirmation signal.

<img width="500" alt="Task 8 Flowchart" src="https://github.com/user-attachments/assets/c269a279-673c-473d-a200-3dbf73e8475a" />

---

## Algorithms

### 1. Global Navigation & Pathfinder Optimization (Task 3)
* **Core Algorithm:** Dijkstra's Algorithm / A* Search
* **Implementation details:** `[ACTION REQUIRED: Explain briefly how your code converts the grid into a node graph, how it updates weights for obstacles, and how it triggers a recalculation loop if a path is blocked.]`

### 2. Adaptive Cruise & Anti-Windup Torque Control (Task 5)
* **Core Algorithm:** PI Velocity Control + Linear Distance Scaling
* **The Math:** Target Velocity is dynamically throttled based on the front ToF distance metric:
    $$V_{target} = V_{max} \times \left( \frac{\text{Distance}_{\text{current}} - \text{Distance}_{\text{stop}}}{\text{Distance}_{\text{detect}} - \text{Distance}_{\text{stop}}} \right)$$
* **Implementation details:** `[ACTION REQUIRED: Describe how resetting the speed integral memory to 0 during a safety stop prevents the robot from violently lunging forward when an airlock door or obstacle clears.]`

### 3. Open Field Heading-Lock Controller (Task 4 & 8 Fallback)
* **Core Algorithm:** IMU Gyro-Stabilized Heading Hold
* **Implementation details:** `[ACTION REQUIRED: Document how taking a static "snapshot" of the IMU yaw angle the millisecond the robot enters an unguided field zone prevents the robot from developing a wandering target angle, correcting wheel slippage dynamically.]`

---

## ⚙️ Hardware Integration & Pin Specifications

The firmware operates on a multi-peripheral sensor-to-actuator matrix mapped as follows:

| Sub-system Component | Microcontroller Pin | Signal / Protocol Type | Operational Logic / Strategy |
| :--- | :--- | :--- | :--- |


---

## 📂 Repository File Architecture

```text
├── .gitignore                      # Excludes local configuration and IDE build directories
├── README.md                       # Primary project documentation and structural guides
└── arduino/                        # Master microcontroller software suite
    ├── Archive/                    # Historical testing snapshots and deprecated modules
    └── Robot_Main/                 # Production source directories (Modular multi-tab IDE project)
        ├── Communication.ino       # Manages incoming/outgoing serial telemetry commands
        ├── Hardware.ino            # Low-level driver configurations for motor channels and VCC rails
        ├── IRCalibration.ino       # Computes sensor normalization bounds for raw surface reflectivity
        ├── Navigation.ino          # Houses motion primitives, line-following loops, and steering PD logic
        ├── PathFinding.ino         # Solves global route calculations via Dijkstra grid node matrices
        ├── Robot_Main.ino          # Global framework entry point; executes setup() and core state switch
        ├── Sensors.ino             # Polling engines for active ToF fields and SPI RFID scanners
        ├── config.h                # Global calibration parameters, target speeds, and controller gains
        ├── pins.h                  # Hardwired microcontroller GPIO and interface channel mappings
        └── robot_state.h           # Defines structural enum variables representing state variables