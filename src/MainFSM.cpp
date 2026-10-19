#include "MainFSM.h"
#include "OperatingFSM.h"
#include "HardwareFunctions.h"
#include "OperatingFSM.h"
#include "definitions.h"
#include "Motor.h"

extern OperatingFSM operatingFSM; // Declare the external instance of OperatingFSM

MainFSM::MainFSM() : currentState(HOMING), timeInState(0.0f) { }

void MainFSM::updateFSM(float motorPosition) {
    this->motorPosition = motorPosition; // Update the current motor position
    // Serial.print("State: ");
    // Serial.println(currentState);
  switch (currentState) {
    case IDLE:
      idle();
      // Transitions
      if (outsideSafeRange() && (isHomed == true)) {
        setState(ERROR);
        setLED(ERROR_LED_PIN); // Turn on error LED
      } else if (readEnableButton() && (isHomed == false)) {
        setState(HOMING);
      } else if (readEnableButton() && (isHomed == true) && (readSwitchDIP() == 0b000)) {
        setState(OPERATING);
        setLED(OPERATING_LED_PIN); // Turn on operating LED
      }
      break;
    case OPERATING:
      operating();
      // Transitions
      if (readEnableButton() == false) {
        setState(IDLE);
        setLED(IDLE_LED_PIN); // Turn on idle LED
      } else if (outsideSafeRange()) {
        setState(ERROR);
        setLED(ERROR_LED_PIN); // Turn on error LED
      }
      break;

    case HOMING:
      homing();
      // Transition back to IDLE after homing is complete
      if (isHomed) {
        setState(IDLE);
        setLED(IDLE_LED_PIN); // Turn on idle LED
      }
      break;

    case ERROR:
      error();
      // Transition
      if (readEnableButton() == false) {
        setState(IDLE);
        setLED(IDLE_LED_PIN); // Turn on idle LED
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
  isHomed = true; // Set to true to simulate homing completion
  setLED(HOMING_LED_PIN); // Turn on homing LED
}

void MainFSM::error() {
  // In the ERROR state, we can implement error handling logic
  // For now, we just stay in the ERROR state until the enable button is released
  move(0); // Stop the motor
}


