#include "Arduino.h"
#include <Adafruit_AS5600.h>
#include "definitions.h"
#include "MotorEncoder.h"
#include "CascadePosVelController.h"
#include "ShaftEncoder.h"
#include "Motor.h"
#include "BackdrivableController.h"

CascadePosVelController controller;
BackdriveableController backdrivableController; 

void setup() {
  Serial.begin(115200);

  shaftEncoderSetup();
  motorEncoderSetup();
  motorSetup();

  while (!homed) {
    home();
    delay(10);
  }
}

void loop() {
  float targetPosition = 0.0f;
  float dt = 0.01f;

  // run the controller update every 10 ms (defined by dt)
  if (millis() % 10 == 0) {
    float currentPosition = (pulses/(2000.0*51.0))*360.0; // Current position from encoder

    // Cascade controller
    // float power = controller.update(targetPosition, currentPosition, dt); 

    // Backdrivable controller
    float magneticEncoderValue = readShaftEncoderADC() - SHAFT_HOME_POSITION; // Adjust for home position
    float magneticEncoderAngle = magneticEncoderValue * MAGNETIC_ENCODER_RESOLUTION; // Convert to float for PID calculation

    float targetError = 0.0;
    float power = backdrivableController.update(targetError, currentPosition, dt, magneticEncoderAngle);

    move(power);
  }
}