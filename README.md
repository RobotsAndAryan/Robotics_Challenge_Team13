# COMP0204 Term 3 Robotics Challenge (2025-2026) 🤖
**Team Number:** 13
**Board ID:** Kayubo

Welcome to our Term 3 robotics project repository! This repo contains all the code for our autonomous surface-navigating, seed-planting robot. 

Our robot uses a Subsumption Architecture driven by a Finite State Machine (FSM). It navigates using Proportional-Derivative (PD) control for line following, Left/Right LiDARs for wall following, an MPU6050 IMU for dead-reckoning and ramp detection, and a VL53L5CX Time-of-Flight imager for obstacle avoidance. It also talks to the arena server over MQTT using the `MiniMessenger` library.

---

## 📂 Repository Structure
We moved away from a massive, unreadable single-file script and modularized the codebase so different behaviors are easy to test and debug. The code used for the final viva and test run is located in the `main` folder.

* `main.ino` - The master loop. Handles the network heartbeat, the safety override "God Loop," and the switch-case FSM.
* `config.h` - Contains all global variables, pin definitions, and PID tuning constants.
* `motion.cpp / .h` - Handles raw motor outputs, PWM voltage clamping, dead-reckoning sensor fusion, and exact IMU pivot turns.
* `sensors.cpp / .h` - Handles the I2C polling for the ToF matrix, LiDARs, and IMU pitch calculations.
* `nav.cpp / .h` - Houses the complex algorithms: the IR center-of-mass line follower and the proportional wall follower.
* `secrets.h` - (Not tracked in git for security) Contains the WiFi SSID, Password, and MQTT broker IPs.

---

## 🛠️ Required Libraries
To compile this code, you need the following libraries installed via the Arduino Library Manager:
* `Motoron` (Pololu Motoron M3S550 Shield)
* `Adafruit MPU6050` (and `Adafruit Unified Sensor`)
* `SparkFun VL53L5CX` (Time of Flight Imager)
* `MFRC522_I2C` (RFID Scanner)
* `Servo` (Standard Arduino Servo library)
* `MiniMessenger` (Course-provided MQTT wrapper)

---

## 🚀 Setup and Execution Instructions

### Hardware Wiring Notes
* **Power:** We are running a 10.9V battery. To power the Arduino Giga R1 directly and prevent sensor brownouts, we created a solder bridge between the `VM` and `AVIN` pins on the Motoron shield. 
* **Voltage Clamping:** Because our N20 motors are rated for 6~7V, the `setMotors()` function mathematically clamps the PWM output to a maximum of 440 (roughly 6V) for base navigation, and 514 (roughly 7V) when extra torque is needed for the ramp(this will be ensured to only be running for the lowest amount of time possible to decrease chances of motor burning out).

### How to Run
1. Clone this repository to your local machine.
2. Create a `secrets.h` file in the same directory as `main.ino` and add the required WiFi/MQTT macros.
3. Open `main.ino` in the Arduino IDE. Select the **Arduino Giga R1 WiFi** board.
4. Compile and upload. 
5. **DO NOT TOUCH THE ROBOT FOR 3 SECONDS AFTER BOOT.** The MPU6050 runs a 200-sample zero-rate bias calibration on startup. Moving it will ruin the dead-reckoning math.
6. The robot starts in a disabled state (LED blinking). Press the physical button on `Pin 2` to toggle `physical_enable` to `true` and begin the autonomous run.

---

## 🗺️ System Diagrams & Flowcharts

### Software Architecture
*(Graders: This diagram shows how our hardware sensors feed into the safety overrides, and how the FSM delegates tasks to the motor controllers.)*

![Software Overview Diagram](./images/architecture.png) 
> **[NOTE TO SELF: DRAW A BLOCK DIAGRAM IN DRAW.IO SHOWING SENSORS -> FSM -> MOTORS AND REPLACE THIS IMAGE LINK]**

### Key Behaviors Flowchart
*(This outlines the decision tree for line following, RFID planting decisions, and the emergency collision/kill-switch interrupts.)*

![FSM Flowchart](./images/flowchart.png)
> **[NOTE TO SELF: DRAW A FLOWCHART SHOWING THE STATES (Base Nav -> Airlock -> Ramp -> Arena) AND REPLACE THIS IMAGE LINK]**

---

## 📊 Testing & Calibration Evidence
Tuning this robot took a massive amount of physical trial and error. Here is our calibration data:

* **Line Sensor Thresholds:** We found that ambient room light gave our black line a value of ~800us and the floor ~400us. We set a hard software noise filter at `> 500us` to successfully calculate the center of mass without jitter.
* **PID Tuning:** * Line Following: `Kp = 5.0`, `Kd = 2.5` (Smooth tracking at 400 PWM base speed).
  * Wall Following: `Kp = 5.0` targeting a 130mm offset.
* **IMU Zero-Rate Bias:** We noticed the robot naturally drifting during open-field dead reckoning. We wrote a calibration loop in `setup()` to average 200 stationary gyro readings. Subtracting this bias (`z_bias`) fixed our asymmetric left/right turns.
* **ToF Noise Filtering:** A single pixel firing a false positive would permanently freeze the robot. We implemented an array check targeting the middle horizontal band (indices 4-11 on a 4x4 resolution matrix) and required `>= 2` simultaneous hits under 200mm to trigger the FSM emergency brake.

---

## 💡 Reflection: What Worked vs. What Didn't

### What Worked Really Well:
* **Subsumption Architecture:** Putting the collision detection and WiFi `messenger.loop()` completely outside the FSM was the best decision we made. Even if the FSM gets stuck looking for a line, the robot will still instantly stop if a wall is detected or the server sends a kill command.
* **Overvolting for the Ramp:** Dynamically raising the PWM clamp from 440 to 514 exactly when the IMU pitch exceeded 12 degrees gave us the perfect amount of torque to crest the ramp without burning out the 6V motors on the flat ground.

### What Didn't Work (And How We Fixed It):
* **Raw Encoder Dead-Reckoning:** Initially, we tried to drive perfectly straight without lines using just encoder ticks. Manufacturing variances in the motors caused the robot to constantly curve. We fixed this by building a **Sensor Fusion** function (`moveStraightDeadReckoning`) that integrates the Z-axis gyro to track heading drift and applies a proportional correction to the wheel speeds to force a straight line.
* **ToF Memory Violations:** We originally requested an 8x8 (64-pixel) resolution from the ToF but only initialized the library at 4x4 (16 pixels). The `for` loop read unallocated memory, causing the FSM to think there was a ghost obstacle permanently blocking the robot. We matched the array bounds to `i < 12` and it worked perfectly.

---

## 🏆 Project Completion Checklist
*This outlines our completion of the official COMP0204 grading rubric tasks.*

- [x] **1. Standard Line Tracking:** PD controller dynamically tracks the line using a center-of-mass algorithm over a 9-channel IR array.
- [x] **2. Intersection & Tag Alignment:** Successfully halts FSM over RFID tags to request airlock clearance via MQTT.
- [x] **3. Solid Grid Navigation:** Navigates grid and executes exact 150-tick payload drops via a 45-degree incremental servo mechanism.
- [x] **4. Open-Field Dead Reckoning:** Navigates unlined gaps using an active Proportional heading-correction loop based on continuous IMU integration.
- [x] **5. Ramped Incline/Decline Control:** Dynamically overvolts motors based on IMU pitch detection (>12 degrees) and holds torque until pitch returns to level (<5 degrees).
- [x] **6. Wall Following:** Maintains a strict 130mm offset using Left/Right TF-Luna LiDAR feedback when floor markings are lost.
- [x] **7. Obstacle Detection and Avoidance:** Safely detects obstacles via ToF and overrides the FSM to halt the motors before a collision occurs.
- [x] **8. Touch-Based Robot Revival:** Uses ToF depth data to map a proportional deceleration curve, dropping motor speed safely at 80mm to coast into the target without a high-impact collision penalty.
