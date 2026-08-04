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
  //   // delay(10);
  // }


  // while (readShaftEncoderADC() > (1010.0f)) {  // Move to -90 degrees before starting the test
  //   float power = controller.update(-90, (readShaftEncoderADC() - zeroShaftPosition) * MAGNETIC_ENCODER_RESOLUTION, 0.01);
  //   move(power);
  //   delay(10);
  // }

  // setLED(HOMING_LED_PIN);  // turn homing led light on to indicate it has homed
}

void loop() {
  pinMode(7, OUTPUT);  // Set pin 7 as output for LED indicator
  float targetPosition = angles[0];  // Set the target position to the first angle in the array
  // int theta = 0;  // Index for the angles array
  float dt = 0.01f;

  float output = 0.0f;
  float error = 0.0f;

  //2d array of 100 elements to store power and error values for analysis
  float powerArray[50][2];  // [0] for power, [1] for error
  int i = 0;
  bool exportData = false;  // Flag to indicate if data has been exported
  bool exportedData = false;  // Flag to indicate if data has been exported

  move(0);  // Ensure motor is stopped before starting the test
  unsigned long starttime = millis();
  while (readSwitchDIP() == 0b001 || readSwitchDIP() == 0b011 || readSwitchDIP() == 0b101) {  // begin testing when a DIP switch is turned on


    if (exportData == true) {  // store power and error every 100 ms for analysis
    //     error = backdrivableController.getError();
    //     output = backdrivableController.getOutput();
    //     powerArray[i][0] = output;  // store output in array
    //     powerArray[i][1] = error;  // store error in array

    //     i++;
    // } else if (i == 100 && !exportData) {  // print power and error values when DIP switch is turned to 0b010
    //     // Serial.begin(115200);
    //     // delay(100);  // Allow time for Serial to initialize
        for (int j = 0; j < i; j++) {
          Serial.print(powerArray[j][1]);
          Serial.print(", ");
          Serial.println(powerArray[j][0]);
        }
        exportData = false;  // set export flag to false to indicate data has been exported
        exportedData = true;  // set exported flag to true to indicate data has been exported
    }

    if (millis() % 2 == 0) {  // run the controller & FSM update every 10 ms (defined by dt)
      motorPosition = (pulses / (2000.0 * 51.0)) * 360.0;  // Current position from encoder
      motorPosition = movingAvg(motorPosition, motorPositionBuffer, MOTOR_POSITION_FILTER_BUFFER_SIZE);
    }

    // run the controller & FSM update every 10 ms (defined by dt)

    if(millis() % 200 == 0) {
      // Serial.print("Motor Position: ");
      Serial.print(motorPosition);
      Serial.print(",");
      Serial.println((readShaftEncoderADC() - zeroShaftPosition) * MAGNETIC_ENCODER_RESOLUTION);
    }

    // if (millis() % 10 == 0) {

    //   float shaftPosition = (readShaftEncoderADC() - zeroShaftPosition) * MAGNETIC_ENCODER_RESOLUTION;  // Current position from shaft encoder

    //   float speed = 1.8310546875 * analogRead(SPEED_PIN) - 740;
    
    //   // float power = controller.update(targetPosition, shaftPosition, dt);
    //   // float power = backdrivableController.update(0, motorPosition, dt, (readShaftEncoderADC()-zeroShaftPosition)*MAGNETIC_ENCODER_RESOLUTION);
    //   float power = springController.updateLinear(0.0, motorPosition, dt, shaftPosition);
    //   // float power = trajectory.followRampTrajectory(shaftPosition, dt, starttime);

    //   // Use the output to control the motor (set PWM value)
    //   move(power);

    //   if (shaftPosition < SAFE_MIN || shaftPosition > SAFE_MAX) {
    //     digitalWrite(ERROR_LED_PIN, HIGH);  // Turn on LED when out of safe range
    //   } else {
    //     digitalWrite(ERROR_LED_PIN, LOW);  // Turn off LED when in safe range
    //   }

      // if (findInArrayWithTolerance(angles, numAngles, shaftPosition, 1.0)) {  // Check if motor angle has reached one of the specified angles in the angles array
      //   digitalWrite(7, HIGH);  // Turn on LED when target angle reached
      //   Serial.print("Compression: ");
      //   Serial.println(motorPosition-shaftPosition);
      // } else {
      //   digitalWrite(7, LOW);  // Turn off LED when not at target angle
      // }

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
}

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