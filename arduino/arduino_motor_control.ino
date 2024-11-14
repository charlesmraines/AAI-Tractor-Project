// Define pins for the stepper motor (used for steering)
const int directionPin = 2; // Pin for controlling the direction of the steering motor
const int stepPin = 3;      // Pin for controlling the steps of the steering motor

// Define pins for the linear actuator (used for acceleration control)
int RPWM = 5;               // Right PWM pin for controlling speed in reverse
int LPWM = 6;               // Left PWM pin for controlling speed in forward direction
int L_EN = 7;               // Left enable pin for the linear actuator
int R_EN = 8;               // Right enable pin for the linear actuator

// Constants for steering control
const int maxSteps = 75000; // Maximum steps for full turn (e.g., full right or full left)
int currentSteps = 0;       // Track current step count for steering

void setup() {
    // Set stepper motor pins for steering as outputs
    pinMode(directionPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    
    // Set linear actuator control pins as outputs
    for (int i = 5; i < 9; i++) {
        pinMode(i, OUTPUT);
        digitalWrite(i, LOW);  // Initialize all linear actuator pins to LOW (off)
    }
    
    Serial.begin(9600); // Begin serial communication at 9600 baud rate
}

void loop() {
    Serial.println(currentSteps); // Print the current steering steps for debugging

    // Check if serial data is available (expecting two floats for linear and angular velocity)
    if (Serial.available() >= 8) {
        float linearVel = 0;    // Variable to store linear velocity for acceleration control
        float angularVel = 0;   // Variable to store angular velocity for steering control

        // Read linear and angular velocities as floats from serial input
        Serial.readBytes((char*)&linearVel, sizeof(linearVel));
        Serial.readBytes((char*)&angularVel, sizeof(angularVel));

        // Control linear actuator for acceleration based on linear velocity
        controlAcceleration(linearVel);

        // Control steering using stepper motor based on angular velocity
        controlSteering(angularVel);
    } else {
        // If no command received, stop both the linear actuator and stepper motor
        stopMotors();
    }
}

// Function to control acceleration using linear actuator based on linear velocity (x)
void controlAcceleration(float linearVel) {
    if (linearVel > 0) {  // Move forward if linearVel is positive
        digitalWrite(L_EN, HIGH);    // Enable forward movement on linear actuator
        digitalWrite(R_EN, HIGH);    // Enable reverse movement on linear actuator
        analogWrite(RPWM, 0);        // Set reverse PWM to 0, ensuring no backward movement
        analogWrite(LPWM, map(linearVel, 0, 1, 128, 255));  // Map linearVel to PWM range for forward speed
        // map(linearVel, 0, 1, 128, 255) scales linearVel (from the range 0 to 1) to a new range between 128 and 255 for PWM control

    } 
    else if (linearVel < 0) {  // Move backward if linearVel is negative
        digitalWrite(L_EN, HIGH);    // Enable forward movement on linear actuator
        digitalWrite(R_EN, HIGH);    // Enable reverse movement on linear actuator
        analogWrite(RPWM, map(abs(linearVel), 0, 1, 128, 255)); // Map absolute linearVel to PWM for reverse speed
        analogWrite(LPWM, 0);        // Set forward PWM to 0, ensuring no forward movement
    } 
    else {
        stopMotors();  // Stop the actuator if linearVel is zero
    }
}

// Function to control steering using stepper motor based on angular velocity (z)
void controlSteering(float angularVel) {
    if (angularVel > 0) {  // Turn right if angularVel is positive
        turnRight(map(angularVel, 0, 1, 50, 250));  // Adjust steps for the right turn based on angularVel
    } 
    else if (angularVel < 0) {  // Turn left if angularVel is negative
        turnLeft(map(abs(angularVel), 0, 1, 50, 250));  // Adjust steps for the left turn based on angularVel
    } 
    else {
        straightenWheel();  // If angularVel is zero, align the steering wheel
    }
}

// Function to turn steering wheel to the right using stepper motor
void turnRight(int stepCount) {
    if (currentSteps < maxSteps) {           // Ensure steps don't exceed max limit for right turn
        digitalWrite(directionPin, HIGH);    // Set direction to right
        for (int i = 0; i < stepCount; i++) {
            stepMotor();                     // Move the stepper motor for specified steps
        }
        currentSteps += stepCount;           // Update current steps
    }
}

// Function to turn steering wheel to the left using stepper motor
void turnLeft(int stepCount) {
    if (currentSteps > -maxSteps) {          // Ensure steps don't exceed max limit for left turn
        digitalWrite(directionPin, LOW);     // Set direction to left
        for (int i = 0; i < stepCount; i++) {
            stepMotor();                     // Move the stepper motor for specified steps
        }
        currentSteps -= stepCount;           // Update current steps
    }
}

// Function to align the steering wheel to the center
void straightenWheel() {
    if (currentSteps > 0) {                  // If current steps are positive, turn left to center
        digitalWrite(directionPin, LOW);     // Set direction to left
        for (int i = 0; i < 50; i++) {
            stepMotor();                     // Move the stepper motor to decrease steps
        }
        currentSteps -= 50;                  // Decrease step count toward center
    } else if (currentSteps < 0) {           // If current steps are negative, turn right to center
        digitalWrite(directionPin, HIGH);    // Set direction to right
        for (int i = 0; i < 50; i++) {
            stepMotor();                     // Move the stepper motor to increase steps
        }
        currentSteps += 50;                  // Increase step count toward center
    }
}

// Function to stop both linear actuator and stepper motor
void stopMotors() {
    digitalWrite(L_EN, LOW);                 // Disable forward movement on linear actuator
    digitalWrite(R_EN, LOW);                 // Disable reverse movement on linear actuator
}

// Function to move the stepper motor one step (used for steering control)
void stepMotor() {
    digitalWrite(stepPin, HIGH);             // Set step pin HIGH to initiate a step
    delayMicroseconds(100);                  // Short delay to allow step
    digitalWrite(stepPin, LOW);              // Set step pin LOW to complete the step
    delayMicroseconds(100);                  // Short delay between steps
}
