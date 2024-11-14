To control the tractor’s steering and acceleration by interpreting `Twist` messages from a Jetson Nano, the following steps and modifications have been implemented in the code. This code enables the Arduino to receive serial commands with linear and angular velocities and control the linear actuator for acceleration and the stepper motor for steering.

### Code Goal and Logic Overview

The main objective is to:
1. **Interpret Linear (`x`) and Angular (`z`) Velocities**: Read `Twist` messages sent by the Jetson Nano, where `x` represents the linear velocity for acceleration (using a linear actuator) and `z` represents the angular velocity for steering (using a stepper motor).
2. **Control the Steering and Acceleration**:
   - For acceleration, use the `linearVel` value to adjust forward or backward movement through PWM signals on a linear actuator.
   - For steering, use the `angularVel` value to control the stepper motor's direction and movement.

### Explanation of Each Step

1. **Reading Serial Data**:
   - The Arduino reads `Twist` messages over serial communication with the Jetson Nano.
   - It expects two `float` values—`linearVel` for acceleration and `angularVel` for steering.
   - These values are read using `Serial.readBytes()` to extract the data in float format.

2. **Control Acceleration**:
   - **Function**: `controlAcceleration()`
   - **Purpose**: Control the linear actuator based on the `linearVel` value.
   - **Logic**:
     - When `linearVel` is positive, the linear actuator is powered to move forward, with PWM signals mapped from `128` to `255`, allowing smooth control over speed.
     - When `linearVel` is negative, the linear actuator is powered to move in reverse with PWM signals mapped for reverse speed.
     - When `linearVel` is zero, the function stops the actuator by turning off both enable pins (`L_EN` and `R_EN`), effectively disabling movement.

3. **Mapping Linear Velocity to PWM Range**:
   - Line: `analogWrite(LPWM, map(linearVel, 0, 1, 128, 255));`
   - **Explanation**:
     - The `map()` function scales `linearVel` (range `0` to `1`) to a PWM range of `128` to `255`.
     - A minimum PWM of `128` ensures there’s enough power to overcome resistance and start moving the actuator, while `255` provides maximum power for full speed.
     - By controlling this range, the motor’s speed increases smoothly with `linearVel`.

4. **Control Steering**:
   - **Function**: `controlSteering()`
   - **Purpose**: Control the stepper motor to turn the steering wheel based on `angularVel`.
   - **Logic**:
     - When `angularVel` is positive, the `turnRight()` function is called, setting the stepper motor direction to right.
     - When `angularVel` is negative, the `turnLeft()` function is called, setting the direction to left.
     - If `angularVel` is zero, the `straightenWheel()` function centers the steering by adjusting the stepper motor back to the center position.

5. **Turn Functions for Steering**:
   - **Functions**: `turnRight()` and `turnLeft()`
   - **Purpose**: Adjust steering wheel direction by controlling the stepper motor.
   - **Logic**:
     - Each function takes a `stepCount` argument, which is mapped based on `angularVel`.
     - `turnRight()` and `turnLeft()` control the direction by setting the `directionPin` (`HIGH` for right, `LOW` for left) and moving the stepper motor by the specified `stepCount`.
     - These functions also update `currentSteps` to keep track of the motor’s position, ensuring it does not exceed maximum limits (`maxSteps`).

6. **Centering the Steering Wheel**:
   - **Function**: `straightenWheel()`
   - **Purpose**: Move the stepper motor back to the center position when `angularVel` is zero.
   - **Logic**:
     - If `currentSteps` is positive, indicating a right turn, the motor turns left until centered.
     - If `currentSteps` is negative, indicating a left turn, the motor turns right until centered.
     - This function ensures the steering is reset to a neutral position when no turn is required.

7. **Stopping the Motors**:
   - **Function**: `stopMotors()`
   - **Purpose**: Disable the linear actuator and stop steering adjustments when no command is received.
   - **Logic**:
     - Both enable pins for the linear actuator (`L_EN` and `R_EN`) are set to `LOW`, stopping all movement.
     - This function serves as a safety measure to ensure the motors do not continue moving if there is no active command.

### Detailed Breakdown of Key Lines

**Line**: `analogWrite(LPWM, map(linearVel, 0, 1, 128, 255));`
- This line sets the speed for forward motion of the linear actuator.
- `map(linearVel, 0, 1, 128, 255)` converts the `linearVel` range (`0` to `1`) to PWM values (`128` to `255`).
- A PWM value of `128` or more provides the necessary power for forward movement, allowing proportional speed control based on `linearVel`.

**Line**: `digitalWrite(directionPin, HIGH); // Set direction to right`
- This line sets the direction of the stepper motor to turn right.
- It is used in `turnRight()` to control the steering direction.

**Line**: `for (int i = 0; i < stepCount; i++) { stepMotor(); }`
- This loop in `turnRight()` and `turnLeft()` moves the stepper motor by `stepCount` steps, enabling control over the turn intensity.
- The `stepMotor()` function toggles the `stepPin` to incrementally adjust the steering angle.

**Line**: `Serial.readBytes((char*)&linearVel, sizeof(linearVel));`
- This reads `linearVel` from serial as a float, interpreting data sent by the Jetson Nano over serial communication.
- The use of `sizeof(linearVel)` ensures the correct number of bytes are read, allowing precise control based on the `Twist` message.

### Summary

This Arduino code allows for:
1. **Receiving Twist Commands**: Linear (`x`) and angular (`z`) velocities from Jetson Nano over serial communication.
2. **Linear Actuator Control**: Smooth forward/backward speed adjustment for acceleration based on `linearVel`.
3. **Stepper Motor Control**: Right/left steering control based on `angularVel`, with centering when idle.
4. **Safety Stop**: Stopping the motors if no command is received, ensuring the tractor remains stationary if communication is lost.

By using `analogWrite` for PWM control of the linear actuator and precise step control of the stepper motor, the code enables responsive and accurate control over the tractor’s acceleration and steering.



The line 

```cpp
analogWrite(LPWM, map(linearVel, 0, 1, 128, 255));
```

is used to control the speed of a motor by adjusting the PWM (Pulse Width Modulation) signal sent to the pin `LPWM` based on the value of `linearVel`.

Here’s a breakdown of what this line does:

1. **`analogWrite(LPWM, value)`**:
   - `analogWrite()` sends a PWM signal to the pin `LPWM`, which allows us to control the power supplied to the motor.
   - PWM values on Arduino range from `0` (off) to `255` (full power).
   - By changing this value, we control the speed of the motor, with higher values resulting in a faster motor speed.

2. **`map(linearVel, 0, 1, 128, 255)`**:
   - The `map()` function in Arduino takes a value within a specified range and scales it to a new range.
   - Here, `linearVel` represents the linear velocity command, expected to be between `0` and `1`.
   - `map(linearVel, 0, 1, 128, 255)` will scale `linearVel` (from the range `0` to `1`) to a new range between `128` and `255`.
     - When `linearVel` is `0`, the mapped value will be `128`.
     - When `linearVel` is `1`, the mapped value will be `255`.
     - Intermediate values of `linearVel` (e.g., `0.5`) will be mapped proportionally within this range (e.g., `192` for `0.5`).

3. **Why Use `128` to `255` Range**:
   - Setting a minimum PWM of `128` ensures that there is enough power to start moving the motor at a reasonable speed. Values below this might not provide enough power to overcome initial resistance.
   - Mapping to `255` at the high end provides the maximum power to the motor for full speed.

In summary, this line adjusts the motor speed based on `linearVel`, ensuring smooth acceleration from a lower limit (`128`) to maximum power (`255`). The motor’s speed will increase proportionally with `linearVel`, allowing finer control over acceleration.





