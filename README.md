# Micromouse

An autonomous maze-solving robot designed to navigate a maze by detecting walls, controlling its motion, and executing turns with feedback from multiple sensors.

The robot combines distance sensing, wheel encoders, and an IMU to control its movement and improve its ability to travel straight and execute accurate turns.

## Demo

![Top View](WhatsApp Image 2026-09-08 at 4.34.10 PM.jpg)
![Bottom View](WhatsApp Image 2026-09-08 at 4.34.10 PM (1).jpg)

## Hardware

* **MCU:** ESP32
* **Motor driver:** TB6612FNG
* **Distance sensing:** 5 × VL53L0X ToF sensors
* **Sensor multiplexer:** TCA9548A
* **Encoders:** 2 × AS5600 magnetic encoders
* **IMU:** MPU6050
* **Motors:** 2 × N20 geared motors
* **Wheels:** 40 mm
* **Power:** 7.4 V Li-ion battery
* **Communication:** I²C

## How It Works

The robot uses multiple VL53L0X ToF sensors to measure the distance to the surrounding walls. The sensors are connected through a TCA9548A I²C multiplexer to allow multiple identical sensors to operate on the same bus.

AS5600 magnetic encoders provide wheel movement feedback, allowing the ESP32 to measure the speed of each motor independently. A closed-loop motor speed controller adjusts the motor PWM to follow the required wheel speeds.

For wall following and centering, the difference between the left and right wall distances is used as the steering error. The controller adjusts the relative speed of the two motors to keep the robot aligned within the corridor.

An MPU6050 gyroscope provides yaw-rate feedback for additional motion stabilization and is also used during turns. Combining encoder feedback, ToF measurements, and gyro feedback allows the robot to control both its translational motion and heading.

The robot's navigation and turning behaviour were developed and tuned through repeated testing on the maze.

## Control System

The robot uses multiple feedback mechanisms:

* **Motor speed control:** Encoder-based closed-loop control
* **Wall centering:** Distance-based steering correction
* **Heading stabilization:** Gyroscope feedback
* **Turn control:** Gyro and encoder-assisted turning
* **Sensor calibration:** Individual sensor and gyro calibration
* **Motor calibration:** Compensation for differences between the two motors

## Challenges & Debugging

### 1. Uncommanded Drift (Hardware Degradation & PID Runaway)

**The Problem:** After a month of inactivity, degradation in the right motor caused the robot to drive straight for 1–2 feet before violently steering into the right wall — with no code changes made in between.

**The Solution:** Traced the fault to the degraded motor rather than the control logic. Replacing the motor and recalibrating resolved the issue, confirming the original PID tuning had been correct all along — the hardware, not the software, had drifted.

### 2. Motor Asymmetry & Open-Loop Calibration

**The Problem:** Out-of-the-box DC motors are never perfectly matched, and after replacing the right motor, the existing PID constants no longer fit the new hardware pairing.

**The Solution:** Wrote a dedicated open-loop calibration script to bypass all navigation logic entirely. By feeding manual PWM values and measuring the response, I calculated exact stiction offsets (`PWM_OFFSET_L/R`) and linear feedforward slopes (`FF_SLOPE_L/R`) for the new motors. This gave the PID controller a mathematically accurate baseline, letting the bot track straight before sensor feedback even engaged.

### 3. Zigzag Blind Spots (Corner Clipping)

**The Problem:** The robot navigated straight corridors cleanly but clipped inner corner pegs during zigzag maneuvers. The original Left/Right/Center ToF layout only looked straight ahead and directly sideways — the 45° front corners of the robot were completely blind.

**The Solution:** Added two diagonal VL53L0X sensors to cover the blind corners. Upgraded the TCA9548A multiplexer's round-robin polling from a 3-state to a 5-state non-blocking switch, so all five sensors could be read continuously without blocking the 100Hz motor control loop.

### 4. Sensor Deadzone Blindness (<40mm Hardware Limit)

**The Problem:** VL53L0X sensors lose accuracy below ~40mm and eventually return a timeout code (8190). Since the steering logic was proportional (`Target − Current Distance`), the correction flatlined or disabled itself right when the robot was inches from a crash.

**The Solution:** Implemented a software "deadzone latch": if a sensor reported a danger-range reading and then suddenly jumped to the 8190 timeout, the system assumed a wall/peg had breached the deadzone and forced the motors into a fixed, aggressive "bang-bang" evasive maneuver instead of trusting the (now-meaningless) proportional value.

### 5. Inertia vs. False Positives (Tuning Hysteresis)

**The Problem:** Setting the evasive trigger distance too short failed outright — at high speeds, the robot's momentum meant it crashed before the wheels could spin down in time. Raising the threshold to a safer distance avoided corner crashes but caused violent swerving on straightaways, since a flat side wall is detected much closer by an angled sensor. Compounding this, raising the danger trigger without properly adjusting the clear trigger broke the hysteresis logic entirely.

**The Solution:** Found the geometric sweet spot just under the straight-wall distance while maintaining a proper hysteresis gap. To compensate for the later trigger point, I added an instant active brake the moment an evasive maneuver starts, giving the tires the grip and time needed to execute a tight pivot instead of skidding into the corner.

### 6. Linear vs. Exponential Correction

**The Problem:** The initial proportional/linear correction produced insufficient response for large positional errors while becoming unnecessarily sensitive around smaller errors. This was particularly noticeable when using the diagonal sensors near angled walls.

**The Solution:** The correction was changed to an exponential response, allowing small deviations to produce gentle corrections while increasing the response more aggressively as the error became larger.

## Results

* **Development Testing:** Successfully and consistently navigated a custom-built test maze during the prototyping phase.
* **Competition Debut:** Competed in Mazerunner Competition conducted by GEC Thrissur, successfully clearing 3 out of 8 corridors on its first official run. 
* **Ongoing Iteration:** The competition highlighted critical edge-cases (such as the zigzag corner-clipping), which directly drove the V2 hardware and software upgrades documented in the challenges above.

The robot was successfully developed from individual hardware components into an autonomous maze-solving platform.

The project involved the development and integration of:

* Sensor-based wall detection
* Encoder-based motor feedback
* Closed-loop motor speed control
* Gyroscope-based heading feedback
* Autonomous turning
* Sensor and motor calibration

The robot was tested and tuned through multiple iterations, with each stage used to identify and correct mechanical, electrical, sensing, and control-related issues.

## Build / Run

The repository contains the firmware and project files required to build and operate the Micromouse.

[Add wiring diagram, pin mapping, and setup instructions here]

### Basic Setup

1. Assemble the chassis, motors, sensors, encoders, and motor driver.
2. Connect the VL53L0X sensors and AS5600 encoders through the TCA9548A multiplexer.
3. Connect the MPU6050 to the ESP32.
4. Connect the TB6612FNG to the ESP32 and motors.
5. Upload the firmware to the ESP32.
6. Perform the required sensor and motor calibration.
7. Test straight-line movement and turning before running the robot in the maze.

## Project Structure

[Add repository folder structure here]
