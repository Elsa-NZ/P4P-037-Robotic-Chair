#include "Arduino.h"
#include <Adafruit_AS5600.h>
#include "definitions.h"
// #include "MotorEncoder.h"
#include "CascadePosVelController.h"
#include "ShaftEncoder.h"
#include "HardwareFunctions.h"
#include "OperatingFSM.h"
#include "MainFSM.h"

CascadePosVelController controller;   
MainFSM mainFSM;
OperatingFSM operatingFSM;

#include "Motor.h"
#include "BackdrivableController.h"

// CascadePosVelController controller;
BackdrivableController backdrivableController; 

void setup() {
  Serial.begin(115200);
  setupHardwareState();
  shaftEncoderSetup();
  motorEncoderSetup();
  motorSetup();

  // digitalWrite(DIRECTION_PIN, HIGH); // HIGH - CCW, LOW - CW

  setLED(IDLE_LED_PIN);
  // while (!homed) {
  //   home();
  //   delay(10);
  // }
}

void loop() {
  // float targetPosition = 0.0f;
  float dt = 0.01f;

  // Serial.print("Shaft Position: ");
  // Serial.println(readShaftEncoderADC());

  // run the controller & FSM update every 10 ms (defined by dt)
  if (millis() % 10 == 0) {
    float motorPosition = (pulses/(2000.0*51.0))*360.0; // Current position from encoder
    // Serial.print("Motor Position: ");
    // Serial.println(motorPosition);
    // Serial.print("Shaft Position: ");
    // Serial.println(readShaftEncoderADC());

    mainFSM.updateFSM(motorPosition);
    // operatingFSM.updateFSM();

    // Serial.println(motorPosition);

    // float power = controller.update(targetPosition, currentPosition, dt);

    //  if (power < 0) {
    //   digitalWrite(DIRECTION_PIN, HIGH); // CW
    //   power = abs(power);
    // } else {
    //   digitalWrite(DIRECTION_PIN, LOW); // CCW
    // }

    // // Use the output to control the motor (set PWM value)
    // analogWrite(PWM_PIN, power);
    // Cascade controller
    // float power = controller.update(targetPosition, currentPosition, dt); 

    // Backdrivable controller
    // float magneticEncoderValue = readShaftEncoderADC() - SHAFT_HOME_POSITION; // Adjust for home position
    // float magneticEncoderAngle = magneticEncoderValue * MAGNETIC_ENCODER_RESOLUTION; // Convert to float for PID calculation

    // float targetError = 0.5;
    // float power = backdrivableController.update(targetError, currentPosition, dt, magneticEncoderAngle);

    // Serial.print("Current Position: ");
    // Serial.print(currentPosition);
    // Serial.print(" | Magnetic Encoder: ");
    // Serial.print(magneticEncoderAngle);
    // Serial.print(" | Power: ");
    // Serial.println(power);

    // move(power);

    // Serial.print("Button State: ");
    // Serial.println(digitalRead(ON_OFF_BUTTON));

  }
}