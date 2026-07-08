#include "Arduino.h"
#include <Adafruit_AS5600.h>
#include "definitions.h"
#include "MotorEncoder.h"
#include "CascadePosVelController.h"
#include "ShaftEncoder.h"
#include "HardwareFunctions.h"
#include "OperatingFSM.h"
#include "MainFSM.h"

CascadePosVelController controller;   
MainFSM mainFSM;
OperatingFSM operatingFSM;


void setup() {
  Serial.begin(115200);

  setupShaftEncoder();
  motorEncoderSetup();
  setupHardwareState();

  pinMode(ENABLE_PIN, OUTPUT); // ENABLE
  pinMode(DIRECTION_PIN, OUTPUT); // DIRECTION
  pinMode(PWM_PIN, OUTPUT); // PWM
  pinMode(SPEED_PIN, INPUT); // SPEED

  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(DIRECTION_PIN, HIGH); // HIGH - CCW, LOW - CW

  setLED(IDLE_LED_PIN);
}

void loop() {
  float targetPosition = 0.0f;
  float dt = 0.01f;

  // Serial.print("Shaft Position: ");
  // Serial.println(readShaftEncoderADC());

  // run the controller & FSM update every 10 ms (defined by dt)
  if (millis() % 10 == 0) {
    float motorPosition = (pulses/(2000.0*51.0))*360.0; // Current position from encoder
    // Serial.print("Motor Position: ");
    // Serial.println(motorPosition);

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
  }
}