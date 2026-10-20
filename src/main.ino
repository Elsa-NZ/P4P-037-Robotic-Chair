// TEST SCRIPT FOR MEASURING FORCES AT SPECIFIED ANGLES
#include <Adafruit_AS5600.h>

#include "Arduino.h"
#include "PositionController.h"
#include "HardwareFunctions.h"
#include "MainFSM.h"
#include "Motor.h"
#include "MotorEncoder.h"
#include "OperatingFSM.h"
#include "ShaftEncoder.h"
#include "TorqueController.h"
#include "Trajectory.h"
#include "definitions.h"

PositionController controller;
TorqueController springController;
MainFSM mainFSM;
OperatingFSM operatingFSM;
Trajectory trajectory;

float motorPosition = 0.0f;

void setup() {
  Serial.begin(115200);
  setupHardwareState();
  shaftEncoderSetup();
  motorEncoderSetup();
  motorSetup();
}

void loop() {
  pinMode(7, OUTPUT);  // Set pin 7 as output for LED indicator
  move(0);  // Ensure motor is stopped before starting the test

  while (true) {  // Run FSM when a DIP switch is turned on

    motorPosition = (pulses / (2000.0 * 51.0)) * 360.0;  // Current position from encoder
    mainFSM.updateFSM(motorPosition);  // Update the main FSM to handle state
                                       // transitions and actions
  }
}