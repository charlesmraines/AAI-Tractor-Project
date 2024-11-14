
# Autonomous Tractor Controller using ROS 2 and Arduino

This project controls an autonomous tractor's steering and acceleration using ROS 2 on a Jetson Nano and an Arduino. The Jetson Nano receives `Twist` commands (linear and angular velocities) from ROS 2, processes them, and sends serial commands to the Arduino to control stepper motors and actuators for steering and acceleration.

## Table of Contents
- [Overview](#overview)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation and Setup](#installation-and-setup)
- [Project Structure](#project-structure)
- [Arduino Code](#arduino-code)
- [ROS 2 Python Node](#ros-2-python-node)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Acknowledgments](#acknowledgments)

## Overview
This system uses:
- A **Jetson Nano** running ROS 2 to send `Twist` commands (linear and angular velocities) as serial data to the Arduino.
- An **Arduino** to control the tractor’s stepper motors and actuators based on the received commands.
  
The `Twist` command’s `linear.x` velocity controls the tractor's forward and backward acceleration, while `angular.z` velocity controls the steering.

## Hardware Requirements
1. **Jetson Nano** - Running ROS 2 and sending `Twist` commands.
2. **Arduino** - Receiving serial commands and controlling actuators and motors.
3. **Stepper Motors** - Used for steering control.
4. **Linear Actuator or Motor** - For acceleration control.
5. **Wiring and Power Supply** - Ensure sufficient power for the motors and actuators.

## Software Requirements
1. **ROS 2 (Foxy or later)** - For running the `TractorMotorController` node on Jetson Nano.
2. **Jetson GPIO** - For Jetson Nano GPIO control.
   ```bash
   pip install Jetson.GPIO
   ```
3. **Arduino IDE** - To upload code to the Arduino.
4. **DepthAI SDK (if needed)** - For any camera integration (optional).
5. **Serial Communication Library (e.g., pyserial)** - For serial communication between Jetson Nano and Arduino.

## Installation and Setup

### 1. ROS 2 Setup on Jetson Nano
- Install ROS 2 on the Jetson Nano and set up a ROS workspace if not already done.
- Place the `TractorMotorController` Python node in your ROS 2 package within the workspace.

### 2. Arduino Setup
- Open the Arduino IDE, upload the provided `Arduino` code to the Arduino board.
- Ensure that the Arduino is properly connected to the Jetson Nano via USB or serial connection.

### 3. Jetson Nano and Arduino Wiring
- **Stepper Motor Wiring**: Connect the Arduino pins for direction and step control to the appropriate pins on the stepper motor driver.
- **Actuator Wiring**: Connect the Arduino PWM pins to the actuators controlling acceleration and braking.

## Project Structure

```
autonomous_tractor_controller/
├── ros2_ws/
│   └── src/
│       └── tractor_motor_controller/
│           ├── tractor_motor_controller_node.py  # ROS 2 Python node on Jetson Nano
├── arduino/
│   └── arduino_motor_control.ino                 # Arduino code for motor and actuator control
└── README.md
```

## Arduino Code

The Arduino code (`arduino_motor_control.ino`) does the following:
- Reads serial data for linear (`x`) and angular (`z`) velocities sent from the Jetson Nano.
- Uses `linear.x` velocity to control the tractor's acceleration (forward/backward).
- Uses `angular.z` velocity to control the tractor's steering direction (left/right).

### Code Highlights
- **Steering Control**: Controls stepper motor direction and steps based on `angular.z` velocity.
- **Acceleration Control**: Controls actuator PWM for forward/backward movement based on `linear.x` velocity.

## ROS 2 Python Node

The ROS 2 node (`tractor_motor_controller_node.py`) on the Jetson Nano:
- Subscribes to the `cmd_vel` topic to receive `Twist` messages (containing `linear.x` and `angular.z`).
- Converts these values into serial commands sent to the Arduino for controlling the tractor.

### Code Highlights
- **Acceleration**: Controlled by `linear.x` (forward, backward, or stop).
- **Steering**: Controlled by `angular.z` (turn left, right, or go straight).

## Usage

1. **Start the ROS 2 Node** on the Jetson Nano:
   ```bash
   ros2 run tractor_motor_controller tractor_motor_controller_node
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

- **Serial Communication Issues**: Ensure the baud rate on both Jetson Nano and Arduino are matched (e.g., `9600` baud).
- **Low Frame Rate for Commands**: Adjust the `Twist` publish rate if the Arduino commands aren’t received smoothly.
- **Power Supply Problems**: Ensure that the motors and actuators are getting adequate power, as voltage drops can affect performance.
- **Direction/Speed Control Issues**: Check the wiring of direction and step control pins for proper connections to the Arduino.

## Acknowledgments
This project utilizes ROS 2 for communication and control  Arduino microcontroller capabilities for autonomous navigation.