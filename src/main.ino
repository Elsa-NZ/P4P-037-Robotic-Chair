// TEST SCRIPT FOR MEASURING FORCES AT SPECIFIED ANGLES
#include <Adafruit_AS5600.h>
#include "Arduino.h"
#include "CascadePosVelController.h"
#include "HardwareFunctions.h"
#include "MainFSM.h"
#include "MotorEncoder.h"
#include "OperatingFSM.h"
#include "ShaftEncoder.h"
#include "definitions.h"
#include "BackdrivableController.h"
#include "Motor.h"
#include "SpringController.h"
#include "Trajectory.h"

CascadePosVelController controller;
BackdrivableController backdrivableController;
SpringController springController;
MainFSM mainFSM;
OperatingFSM operatingFSM;
Trajectory trajectory;

// // Angles from -45 to 25 degrees
float angles[] = {-90.0, -85.0, -80.0, -75.0, -70.0, -65.0, -60.0, -55.0, -50.0, -45.0, 
                  -40.0, -35.0, -30.0, -25.0, -20.0,  -15.0, -10.0,
                  -5.0,  0.0,  5.0,  10.0,  15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0};
// Angles from -45 to 25 degrees
// float angles[] = {-45.0};
// float angles[] = {0.0};
int numAngles = sizeof(angles) / sizeof(angles[0]);

// moving average buffer for motor position
float motorPositionBuffer[MOTOR_POSITION_FILTER_BUFFER_SIZE];
int motorPositionBufferIndex = 0;
float motorPosition = 0.0f;



void setup() {
  Serial.begin(115200);
  setupHardwareState();
  shaftEncoderSetup();
  motorEncoderSetup();
  motorSetup();

  // Home at the very start of testing!!
  // while (!homed) {
  //   home();
  // }

  // setLED(HOMING_LED_PIN);  // turn homing led light on to indicate it has homed
}

void loop() {
  pinMode(7, OUTPUT);  // Set pin 7 as output for LED indicator
  float targetPosition = 0;  // Set the target position to the first angle in the array
  float targetTorque = 0;  // Set the target torque to the first angle in the array
  double lastSpeed = 0;  // Variable to store the last speed for acceleration calculation
  float dt = 0.01f;

  float output = 0.0f;
  float error = 0.0f;

  unsigned long starttime = millis();
  double currentfreq = 0; // starting frequency 0 Hz
  double currentphase = 0; // starting phase 0 rad

  move(0);  // Ensure motor is stopped before starting the test

  float startPosition1 = 0.0f;
  float startPosition2 = 0.0f;
  float startPosition3 = 0.0f;

  while (true) {  // Run FSM when a DIP switch is turned on

    // if (millis() % 2 == 0) {  // run the controller & FSM update every 10 ms (defined by dt)
      motorPosition = (pulses / (2000.0 * 51.0)) * 360.0;  // Current position from encoder
      motorPosition = movingAvg(motorPosition, motorPositionBuffer, MOTOR_POSITION_FILTER_BUFFER_SIZE);
      mainFSM.updateFSM(motorPosition);  // Update the main FSM to handle state transitions and actions
    // }

    // if (millis() % 10 == 0) {

      // float shaftPosition = (readShaftEncoderADC() - zeroShaftPosition) * MAGNETIC_ENCODER_RESOLUTION;  // Current position from shaft encoder
  
      // float speed = ((constrain(analogRead(SPEED_PIN) * (5.0/1023.0), V_min, V_max) / V_max) * (RPM_max - RPM_min) + RPM_min) * 6.0 / 51.0 ; // Convert analog reading to speed in RPM
      // double acceleration = (speed - lastSpeed) / dt;
      // lastSpeed = speed;

      // // generate chirp sweep signal for target position
      // float t = (millis() - starttime) / 1000.0; // time in seconds
      // currentfreq = 0.0f + k * t; 
      // currentphase = currentphase + 2 * PI * currentfreq * dt;
      // // targetPosition = amplitude * sin(currentphase); // target position is a sine wave with amplitude of 45 degrees
      // targetTorque = amplitude * sin(currentphase); // target position is a sine wave with amplitude of 45 degrees

      // float power = controller.update(targetPosition, shaftPosition, dt, currentfreq, currentphase);
      // float power = backdrivableController.update(6.5, motorPosition, dt, shaftPosition);
      // float power = springController.updateLinear(0.15, 0, motorPosition, dt, shaftPosition);
      // float power = springController.torqueController(motorPosition, dt, shaftPosition, 4.0); // target torque is a sine wave with amplitude of 1 Nm
      // float power = trajectory.followCurveTrajectory(shaftPosition, dt, starttime);
      // float power = 0.0f;
      
      // Implement S-curve trajectory for torque control
      // if (millis() - starttime < 12000) {
      //   power = springController.SCurveTraj(0.0f, 10.0f, 10000.0f, dt, starttime, motorPosition, shaftPosition); 
      // } else if (millis() - starttime >= 12000 && millis() - starttime < 17000) {
      //   power = springController.SCurveTraj(10.0f, -5.0f, 3000.0f, dt, starttime+12000, motorPosition, shaftPosition);
      // } else if (millis() - starttime >= 17000 && millis() - starttime < 20000) {
      //   power = springController.SCurveTraj(-5.0f, 0.0f, 1000.0f, dt, starttime+17000, motorPosition, shaftPosition);
      // } else if (millis() - starttime >= 20000) {
      //   power = springController.SCurveTraj(0.0f, 1.0f, 2000.0f, dt, starttime+20000, motorPosition, shaftPosition);
      // }

      // Implement S-curve trajectory for position control
      // if (millis() - starttime < 12000) {
      //   power = trajectory.SCurveTraj(0.0f, -50.0f, 10000.0f, dt, starttime, shaftPosition); 
      // } else if (millis() - starttime >= 12000 && millis() - starttime < 17000) {
      //   power = trajectory.SCurveTraj(-50.0f, -5.0f, 3000.0f, dt, starttime+12000, shaftPosition);
      // } else if (millis() - starttime >= 17000 && millis() - starttime < 20000) {
      //   power = trajectory.SCurveTraj(-5.0f, 20.0f, 1000.0f, dt, starttime+17000, shaftPosition);
      // } else if (millis() - starttime >= 20000) {
      //   power = trajectory.SCurveTraj(20.0f, 0.0f, 2000.0f, dt, starttime+20000, shaftPosition);
      // }

      // if (millis() - starttime < 5000) {
      //   power = backdrivableController.update(6.5, motorPosition, dt, shaftPosition);   // Store the starting position for the next trajectory
      // } else if (millis() - starttime >= 5000) {
      //   
      //   power = backdrivableController.update(targetTorque, motorPosition, dt, shaftPosition);
      // } 

      // float springConst = 0.3;

      // if (millis() - starttime < 7000) {
      //   power = springController.SCurveTraj(springConst, 0.0f, -60.0f, 6000.0f, dt, starttime, motorPosition, shaftPosition);  
      // } else if (millis() - starttime >= 7000 && millis() - starttime < 9500) {
      //   power = springController.SCurveTraj(springConst, -60.0f, 10.0f, 1500.0f, dt, starttime+7000, motorPosition, shaftPosition);
      // } else if (millis() - starttime >= 9500 && millis() - starttime < 11000) {
      //   power = springController.SCurveTraj(springConst, 10.0f, 15.0f, 500.0f, dt, starttime+9500, motorPosition, shaftPosition);
      // } else if (millis() - starttime >= 11000) {
      //   power = springController.SCurveTraj(springConst, 15.0f, -10.0f, 1000.0f, dt, starttime+11000, motorPosition, shaftPosition);
      // }

      // Serial.print(">Target Position: ");
      // Serial.println(targetPosition);
      // Serial.print(">Shaft Position: ");
      // Serial.println(shaftPosition);
      // Serial.print(">Equilibrium Position: ");
      // Serial.println(targetPosition);
      // // Serial.print(">Current Velocity: ");
      // // Serial.println(speed);
      // // Serial.print(">Current Acceleration: ");
      // // Serial.println(acceleration);

      // Serial.print(">Power Output: ");
      // Serial.println(power);

      // Use the output to control the motor (set PWM value)

      // move(power);

      // if (shaftPosition < SAFE_MIN || shaftPosition > SAFE_MAX) {
      //   digitalWrite(ERROR_LED_PIN, HIGH);  // Turn on LED when out of safe range
      // } else {
      //   digitalWrite(ERROR_LED_PIN, LOW);  // Turn off LED when in safe range
      // }

      // UNCOMMENT WHEN U WANT TO FIND SPECIFIC ANGLES
      // if (findInArrayWithTolerance(angles, numAngles, shaftPosition, 1.0)) {  // Check if motor angle has reached one of the specified angles in the angles array
      //   digitalWrite(7, HIGH);  // Turn on LED when target angle reached
      //   // Serial.print("Compression: ");
      //   // Serial.println(motorPosition-shaftPosition);
      // } else {
      //   digitalWrite(7, LOW);  // Turn off LED when not at target angle
      // }
    }

      // if (shaftPosition >= targetPosition - 0.5 && shaftPosition <= targetPosition + 0.5 && speed < 0.5 && exportedData == false) {  // Check if motor angle has reached one of the specified angles and speed is low
      //   digitalWrite(7, HIGH);  // Turn on LED when target angle reached
      //   // delay(500);  // Wait for 500 ms to ensure stable reading
      //   if (i >= numAngles) {
      //     exportData = true;  // Set export flag to true when all angles are reached
      //     move(0.1*255);  // Stop the motor after reaching all angles
      //   }
      //   powerArray[i][0] = motorPosition;  // store motor position in array
      //   powerArray[i][1] = shaftPosition;  // store shaft position in array
      //   Serial.print("Compression: ");
      //   Serial.println(motorPosition-shaftPosition);
      //   // i++;  // Move to the next angle in the array
      //   targetPosition = angles[i];  // Update the target position to the next angle
      // } else {
      //   digitalWrite(7, LOW);  // Turn off LED when not at target angle
      // }
    // }
}
// }

// Helper function check if motor angle has reached one of the specified angles
// in the angles array. If it has, it turns on the operating LED. (do it as
// quickly as possible)

bool findInArrayWithTolerance(float arr[], int size, float value,
                              float tolerance) {
  for (int i = 0; i < size; i++) {
    if (fabs(arr[i] - value) <= tolerance) {
      return true;  // Match found within tolerance
    }
  }
  return false;
}

float movingAvg(float newValue, float buffer[], int bufferSize) {
  // Add the new value to the buffer
  buffer[motorPositionBufferIndex] = newValue;
  motorPositionBufferIndex = (motorPositionBufferIndex + 1) % bufferSize;

  // Calculate the average
  float sum = 0.0f;
  for (int i = 0; i < bufferSize; i++) {
    sum += buffer[i];
  }
  return sum / bufferSize;
}