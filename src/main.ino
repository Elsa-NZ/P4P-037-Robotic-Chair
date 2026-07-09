// TEST SCRIPT FOR MEASURING FORCES AT SPECIFIED ANGLES

// Angles from -25 to 45 degrees
float angles[] = {-25.0, -20.0, -15.0, -10.0, -5.0, 0.0,  5.0, 10.0,
                  15.0,  20.0,  25.0,  30.0,  35.0, 40.0, 45.0};
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
  float targetPosition = 0.0f;
  float dt = 0.01f;
  while (readSwitchDIP() !=
         0b000) {  // begin testing when a DIP switch is turned on

    // Serial.print("Shaft Position: ");
    // Serial.println(readShaftEncoderADC());

    // run the controller & FSM update every 10 ms (defined by dt)
    if (millis() % 10 == 0) {
      float motorPosition =
          (pulses / (2000.0 * 51.0)) * 360.0;  // Current position from encoder
      // Serial.print("Motor Position: ");
      // Serial.println(motorPosition);
      // Serial.print("Shaft Position: ");
      // Serial.println(readShaftEncoderADC());

      // INSERT CONTROLLER HERE //

      // float power = controller.update(targetPosition, motorPosition, dt);

      ////////////////////////////////////////////

      // if (power < 0) {
      //   digitalWrite(DIRECTION_PIN, HIGH);  // CW
      //   power = abs(power);
      // } else {
      //   digitalWrite(DIRECTION_PIN, LOW);  // CCW
      // }

      // // Use the output to control the motor (set PWM value)
      // move(power);

      if (findInArrayWithTolerance(angles, numAngles, motorPosition, 0.1f)) {
        digitalWrite(IDLE_LED_PIN, HIGH);  // Turn on LED when target angle reached
      } else {
        digitalWrite(IDLE_LED_PIN, LOW);  // Turn off LED when not at target angle
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