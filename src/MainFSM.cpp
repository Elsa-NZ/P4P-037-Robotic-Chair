#include "MainFSM.h"
#include "OperatingFSM.h"
#include "HardwareFunctions.h"
#include "OperatingFSM.h"
#include "definitions.h"
#include "Motor.h"
#include "MotorEncoder.h"

extern OperatingFSM operatingFSM; // Declare the external instance of OperatingFSM
MainFSM::MainFSM() : currentState(HOMING), timeInState(0.0f) { }

void MainFSM::updateFSM(float motorPosition) {
    this->motorPosition = motorPosition; // Update the current motor position
  switch (currentState) {
    case IDLE:
      idle();
      // Transitions
      if (outsideSafeRange() && (homed == true)) {
        setState(ERROR);
        LEDOn(ERROR_LED_PIN); // Turn on error LED
      } else if (readEnableButton() && (homed == false)) {
        setState(HOMING);
      } else if (readEnableButton() && (homed == true) && (readSwitchDIP() == 0b000)) {
        setState(OPERATING);
        LEDOff(IDLE_LED_PIN); // Turn off idle LED
        LEDOn(OPERATING_LED_PIN); // Turn on operating LED
      }
      break;
    case OPERATING:
      operating();
      // Transitions
      if (readEnableButton() == false) {
        setState(IDLE);
        LEDOff(OPERATING_LED_PIN); // Turn off operating LED
        LEDOff(HOMING_LED_PIN); // Turn off homing LED
        LEDOn(IDLE_LED_PIN); // Turn on idle LED
        homed = false; // Reset homing status when enable off
      } else if (outsideSafeRange()) {
        setState(ERROR);
        LEDOff(OPERATING_LED_PIN); // Turn off operating LED
        LEDOn(ERROR_LED_PIN); // Turn on error LED
      }
      break;

    case HOMING:
      homing();
      // Transition back to IDLE after homing is complete
      if (homed) {
        setState(IDLE);
        LEDOn(IDLE_LED_PIN); // Turn on idle LED
      }
      break;

    case ERROR:
      error();
      // Transition
      if (readEnableButton() == false) {
        setState(IDLE);
        LEDOff(ERROR_LED_PIN); // Turn off error LED
        LEDOff(HOMING_LED_PIN); // Turn off homing LED
        LEDOn(IDLE_LED_PIN); // Turn on idle LED
        homed = false; // Reset homing status when enable off
      }
      break;
  }
}

void MainFSM::setState(State newState) {
  currentState = newState;
  timeInState = 0.0f;  // Reset time in state on transition
}

bool MainFSM::outsideSafeRange() {
  // Check if the motor position is outside the safe range
  return (motorPosition < SAFE_MIN || motorPosition > SAFE_MAX);
}

// STATE FUNCTIONS

void MainFSM::idle() {
  // do nothing lol
  move(0);
}

void MainFSM::operating() {
  // In the OPERATING state, we can call the operating FSM's update function
  operatingFSM.updateFSM();
}

void MainFSM::homing() {
  // In the HOMING state, we can implement the homing logic
  while (!homed) {
    home();
    // delay(10);
  }
  LEDOn(HOMING_LED_PIN); // Turn on homing LED
}

void MainFSM::error() {
  // In the ERROR state, we can implement error handling logic
  // For now, we just stay in the ERROR state until the enable button is released
  move(0); // Stop the motor
}


