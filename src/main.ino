// TEST SCRIPT FOR MEASURING FORCES AT SPECIFIED ANGLES

// Angles from -45 to 25 degrees
float angles[] = {-45.0, -40.0, -35.0, -30.0, -25.0, -20.0,  -15.0, -10.0,
                  -5.0,  0.0,  5.0,  10.0,  15.0, 20.0, 25.0};
int numAngles = sizeof(angles) / sizeof(angles[0]);

#include <Adafruit_AS5600.h>
#include "Arduino.h"
#include "CascadePosVelController.h"
#include "HardwareFunctions.h"
#include "MainFSM.h"
#include "MotorEncoder.h"
#include "OperatingFSM.h"
#include "ShaftEncoder.h"
#include "definitions.h"

CascadePosVelController controller;
MainFSM mainFSM;
OperatingFSM operatingFSM;

#include "BackdrivableController.h"
#include "Motor.h"

// CascadePosVelController controller;
BackdrivableController backdrivableController;

void setup() {
  Serial.begin(115200);
  setupHardwareState();
  shaftEncoderSetup();
  motorEncoderSetup();
  motorSetup();

  // Home at the very start of testing!!
  while (!homed) {
    home();
    delay(10);
  }

  setLED(HOMING_LED_PIN);  // turn homing led light on to indicate it has homed
}

void loop() {
  pinMode(7, OUTPUT);  // Set pin 7 as output for LED indicator
  // float targetPosition = 0.0f;
  float dt = 0.01f;

  float output = 0.0f;
  float error = 0.0f;

  //2d array of 100 elements to store power and error values for analysis
  float powerArray[100][2];  // [0] for power, [1] for error
  int i = 0;
  bool exportData = false;  // Flag to indicate if data has been exported

  move(0);  // Ensure motor is stopped before starting the test
  while (readSwitchDIP() == 0b001 || readSwitchDIP() == 0b011 || readSwitchDIP() == 0b101) {  // begin testing when a DIP switch is turned on

    // Serial.print("Shaft Position: ");
    // Serial.println(readShaftEncoderADC());

    if (readSwitchDIP() == 0b011 && millis() % 300 == 0 && i < 100) {  // store power and error every 100 ms for analysis
        error = backdrivableController.getError();
        output = backdrivableController.getOutput();
        powerArray[i][0] = output;  // store output in array
        powerArray[i][1] = error;  // store error in array

        i++;
    } else if (i == 100 && !exportData) {  // print power and error values when DIP switch is turned to 0b010
        // Serial.begin(115200);
        // delay(100);  // Allow time for Serial to initialize
        for (int j = 0; j < i; j++) {
          Serial.print(powerArray[j][1]);
          Serial.print(", ");
          Serial.println(powerArray[j][0]);
        }
        exportData = true;  // set export flag to true to indicate data has been exported
    }

    // run the controller & FSM update every 10 ms (defined by dt)
    if (millis() % 10 == 0) {
      float motorPosition = (pulses / (2000.0 * 51.0)) * 360.0;  // Current position from encoder
      Serial.println(motorPosition);

      // float power = controller.update(targetPosition, motorPosition, dt);
      float power = backdrivableController.update(4, motorPosition, dt, (readShaftEncoderADC()-zeroShaftPosition)*MAGNETIC_ENCODER_RESOLUTION);

      // Use the output to control the motor (set PWM value)
      move(power);

      if (motorPosition < SAFE_MIN || motorPosition > SAFE_MAX) {
        digitalWrite(ERROR_LED_PIN, HIGH);  // Turn on LED when out of safe range
      } else {
        digitalWrite(ERROR_LED_PIN, LOW);  // Turn off LED when in safe range
      }

      if (findInArrayWithTolerance(angles, numAngles, motorPosition, 0.1f)) {
        digitalWrite(7, HIGH);  // Turn on LED when target angle reached
      } else {
        digitalWrite(7, LOW);  // Turn off LED when not at target angle
      }
    }
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