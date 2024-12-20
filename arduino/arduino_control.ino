// IMU Libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
//IMU Library inclusion done



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

const int numReadings = 10;

int readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int total = 0;              // the running total
int average = 0;            // the average

void setup() {
    // Set stepper motor pins for steering as outputs
    sensors_event_t orientationData;
    pinMode(directionPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(A1, INPUT);
    
    // Set linear actuator control pins as outputs
    for (int i = 5; i < 9; i++) {
        pinMode(i, OUTPUT);
        digitalWrite(i, LOW);  // Initialize all linear actuator pins to LOW (off)
    }
    
    
    Serial.begin(9600); // Begin serial communication at 9600 baud rate
 
 
 // void setup Code for IMU start
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
        readings[thisReading] = 0;
    }

    // void setup Code for IMU start
    while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
 //void Setup Code for IMU done
  
}

void loop() {

  // void loop IMU code starts
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  printEvent(&orientationData); // A function for printing the data od the IMU which has been declared in later part
  delay(BNO055_SAMPLERATE_DELAY_MS);
  // void loop IMU code ends 

  // Check if there is serial data available
  delay(10);
  if (Serial.available()) {
      // Read the entire line as a string (format: "linearVel angularVel")
      String input = Serial.readStringUntil('\n');
      
      // Find the space in the input and split the string into two parts
      float linearVel;
      float angularVel;
      int spaceIndex = input.indexOf(' ');
      int feedback = smooth_feedback();
      Serial.println(String(feedback));
      
      if (spaceIndex != -1) {
          // Extract and convert the linear velocity
          linearVel = input.substring(0, spaceIndex).toFloat();
          
          // Extract and convert the angular velocity
          angularVel = input.substring(spaceIndex + 1).toFloat();

          //Serial.println("Linear Vel: " + String(linearVel) + ", Angular Vel: " + String(angularVel) + "\n");
          controlAcceleration(linearVel, feedback);
          controlSteering(angularVel);
      }
  } else {
      // If no command, stop both motors
      digitalWrite(L_EN, LOW);
      digitalWrite(R_EN, LOW);
  }
}

// Function for printing IMU data starts
void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
  }
    else {
    Serial.print("Unk:");
  }

  Serial.println(x);
}
// Function for printing IMU data ends.

// Function to smooth feedback
int smooth_feedback()
{
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(A1);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  return average;
}

// Added debug macros to print information
//#define DEBUG

// Updated controlAcceleration function with verbose output
void controlAcceleration(float linearVel, int feedback) {
    const int deadband = 5;  // Tolerance range to ignore small fluctuations
    int mapping_value = 0;

    if (linearVel >= 0.0) {
        mapping_value = map(linearVel, 0.0, 5.0, 312, 600);  // Adjust for forward motion
    } else {
        mapping_value = map(linearVel, -5.0, 0.0, 120, 312);  // Adjust for reverse motion
    }

    #ifdef DEBUG
    Serial.print("Feedback: ");
    Serial.print(feedback);
    Serial.print(", Mapped Value: ");
    Serial.print(mapping_value);
    Serial.print(", Deadband: ");
    Serial.println(deadband);
    #endif

    if (abs(feedback - mapping_value) > deadband) {
        if (feedback > mapping_value) {
            #ifdef DEBUG
            Serial.println("Moving motor: Decreasing feedback");
            #endif
            digitalWrite(L_EN, HIGH);
            digitalWrite(R_EN, HIGH);
            analogWrite(RPWM, 128);  // Adjust speed as needed
            analogWrite(LPWM, 0);
        } else {
            #ifdef DEBUG
            Serial.println("Moving motor: Increasing feedback");
            #endif
            digitalWrite(L_EN, HIGH);
            digitalWrite(R_EN, HIGH);
            analogWrite(RPWM, 0);
            analogWrite(LPWM, 128);
        }
    } else {
        #ifdef DEBUG
        Serial.println("Within deadband: Stopping motor");
        #endif
        digitalWrite(L_EN, LOW);
        digitalWrite(R_EN, LOW);
    }
}

// Function to control the steering based on angular velocity
void controlSteering(float angularVel) {
    // Calculate the number of steps based on the angular velocity
    int steps = map(angularVel, -2.0, 2.0, -maxSteps, maxSteps);

    // Control the direction of the stepper motor based on the sign of the angular velocity
    if (steps > 0) {
        // Turning right (positive angular velocity)
        digitalWrite(directionPin, HIGH);  // Set direction to right
    } else if (steps < 0) {
        // Turning left (negative angular velocity)
        digitalWrite(directionPin, LOW);   // Set direction to left
    }

    // Step the motor based on the calculated number of steps
    currentSteps += steps;
    currentSteps = constrain(currentSteps, -maxSteps, maxSteps);  // Ensure within bounds

    // Make the motor step the calculated number of times
    for (int i = 0; i < abs(steps); i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(100);  // Adjust step timing as needed
        digitalWrite(stepPin, LOW);
        delayMicroseconds(100);  // Adjust step timing as needed
    }
}
