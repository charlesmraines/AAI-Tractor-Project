Here’s the rewritten README incorporating the additional logic for obstacle detection from the camera and its integration into the system:

---

# Autonomous Tractor Controller using ROS 2, Arduino, and Obstacle Detection

This project controls an autonomous tractor's steering and acceleration using ROS 2 on a Jetson Nano and an Arduino. The system also integrates obstacle detection using a DepthAI camera to ensure safe navigation. The Jetson Nano receives `Twist` commands (linear and angular velocities) from ROS 2, processes them, and sends serial commands to the Arduino to control stepper motors and actuators for steering and acceleration. Additionally, the system stops the tractor when an obstacle is detected within a 5-meter range.

## Table of Contents
- [Overview](#overview)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation and Setup](#installation-and-setup)
- [Project Structure](#project-structure)
- [Arduino Code](#arduino-code)
- [ROS 2 Python Nodes](#ros-2-python-nodes)
  - [Tractor Motor Controller Node](#tractor-motor-controller-node)
  - [DepthAI Camera Node](#depthai-camera-node)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Acknowledgments](#acknowledgments)

## Overview
This system integrates:
- A **Jetson Nano** running ROS 2 to manage communication and control logic.
- An **Arduino** to control the tractor’s stepper motors and actuators based on received commands.
- A **DepthAI Camera** to detect obstacles and publish the distance to the nearest object, ensuring safe navigation.
  
### Features:
1. **Navigation**: The `cmd_vel` topic drives the tractor using linear (`x`) and angular (`z`) velocities.
2. **Obstacle Avoidance**: The camera publishes distance data to the `camera/distance` topic. If an obstacle is within 5 meters, the tractor stops.

## Hardware Requirements
1. **Jetson Nano** - Running ROS 2 and sending `Twist` commands.
2. **Arduino** - Receiving serial commands and controlling actuators and motors.
3. **Stepper Motors** - Used for steering control.
4. **Linear Actuator or Motor** - For acceleration control.
5. **DepthAI Camera** - For obstacle detection.
6. **Wiring and Power Supply** - Ensure sufficient power for the motors, actuators, and camera.

## Software Requirements
1. **ROS 2 (Foxy or later)** - For running the ROS nodes.
2. **Jetson GPIO** - For Jetson Nano GPIO control.
   ```bash
   pip install Jetson.GPIO
   ```
3. **DepthAI SDK** - For DepthAI camera integration.
4. **Arduino IDE** - To upload code to the Arduino.
5. **Serial Communication Library (e.g., pyserial)** - For serial communication between Jetson Nano and Arduino.

## Installation and Setup

### 1. ROS 2 Setup on Jetson Nano
- Install ROS 2 on the Jetson Nano and set up a ROS workspace.
- Add the `TractorMotorController` and `DepthAIPublisherNode` Python nodes to your ROS 2 package within the workspace.

### 2. Arduino Setup
- Upload the provided Arduino code (`arduino_motor_control.ino`) to the Arduino board.
- Ensure the Arduino is properly connected to the Jetson Nano via USB or serial connection.

### 3. Jetson Nano and Arduino Wiring
- **Stepper Motor Wiring**: Connect the Arduino pins for direction and step control to the appropriate pins on the stepper motor driver.
- **Actuator Wiring**: Connect the Arduino PWM pins to the actuators controlling acceleration and braking.

### 4. Camera Setup
- Connect the DepthAI camera to the Jetson Nano via USB.
- Ensure the camera is properly mounted and functional.

## Project Structure

```
autonomous_tractor_controller/
├── ros2_ws/
│   └── src/
│       ├── tractor_motor_controller/
│       │   ├── gps_controller_with_camera_node.py  # ROS 2 Python node for overall tractor control based on sensor input.
│       └── depthai_publisher/
│           ├── depthai_publisher_node.py        # ROS 2 Python node for 
camera
│   └── arduino_motor_control.ino                 # Arduino code for motor and 
└── README.md
```

## Arduino Code

The Arduino code (`arduino_motor_control.ino`) does the following:
- Reads serial data for linear (`x`) and angular (`z`) velocities sent from the Jetson Nano.
- Uses `linear.x` velocity to control the tractor's acceleration (forward/backward).
- Uses `angular.z` velocity to control the tractor's steering direction (left/right).

## ROS 2 Python Nodes

### Tractor Motor Controller Node
This ROS 2 node:
- Subscribes to the `cmd_vel` topic to receive `Twist` messages (containing `linear.x` and `angular.z`).
- Converts these values into serial commands sent to the Arduino for controlling the tractor.

#### Key Features:
- **Acceleration**: Controlled by `linear.x` (forward, backward, or stop).
- **Steering**: Controlled by `angular.z` (turn left, right, or go straight).

### DepthAI Camera Node
This ROS 2 node:
- Publishes RGB and depth data from the DepthAI camera.
- Publishes the distance to the nearest obstacle to the `camera/distance` topic.

#### Key Features:
- **Obstacle Detection**: Calculates the distance to the nearest object and publishes it as a `Float32` message.
- **Integration with Control Logic**: The camera's distance data is used to stop the tractor if an obstacle is detected within 5 meters.

## Usage

1. **Start the ROS 2 Nodes** on the Jetson Nano:
   ```bash
   # Start the tractor motor controller
   ros2 run tractor_motor_controller tractor_motor_controller_node

   # Start the DepthAI camera node
   ros2 run depthai_publisher depthai_publisher_node
   ```

2. **Run the Arduino Code**: Upload the Arduino code to the Arduino board and ensure it’s connected to the Jetson Nano.

3. **Control the Tractor**:
   - Publish `Twist` commands on the `cmd_vel` topic with linear and angular velocities to control the tractor.
   - Example command to publish `Twist`:
     ```bash
     ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
     ```
   - Here, `linear.x = 1.0` moves the tractor forward, and `angular.z = 1.0` turns the tractor right.

## Troubleshooting

- **Obstacle Detection Not Working**: Ensure the camera node is publishing to the `camera/distance` topic.
- **Tractor Stops Unexpectedly**: Check if the obstacle detection logic is triggering a stop due to objects within 5 meters.
- **Serial Communication Issues**: Ensure the baud rate on both Jetson Nano and Arduino are matched (e.g., `9600` baud).
- **Power Supply Problems**: Verify adequate power for motors, actuators, and the camera.

## Acknowledgments
This project integrates ROS 2, DepthAI camera, and Arduino microcontroller capabilities to create a robust autonomous tractor navigation system.