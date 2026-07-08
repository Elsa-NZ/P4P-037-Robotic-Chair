#include "MainFSM.h"
#include "OperatingFSM.h"
#include "HardwareFunctions.h"
#include "OperatingFSM.h"
#include "definitions.h"

extern OperatingFSM operatingFSM; // Declare the external instance of OperatingFSM

MainFSM::MainFSM() : currentState(IDLE), timeInState(0.0f) { }

void MainFSM::updateFSM(float motorPosition) {
    this->motorPosition = motorPosition; // Update the current motor position
    Serial.print("State: ");
    Serial.println(currentState);
  switch (currentState) {
    case IDLE:
      idle();
      // Transitions
      if (outsideSafeRange()) {
        setState(ERROR);
        setLED(ERROR_LED_PIN); // Turn on error LED
      } else if (readEnableButton() && (isHomed == false)) {
        setState(HOMING);
        setLED(HOMING_LED_PIN); // Turn on homing LED
      } else if (readEnableButton() && (isHomed == true)) {
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
}

void MainFSM::operating() {
  // In the OPERATING state, we can call the operating FSM's update function
  operatingFSM.updateFSM();
}

void MainFSM::homing() {
  // In the HOMING state, we can implement the homing logic
  isHomed = true; // Set to true to simulate homing completion
  delay(3000); // Simulate some time taken for homing
}

void MainFSM::error() {
  // In the ERROR state, we can implement error handling logic
  // For now, we just stay in the ERROR state until the enable button is released
}


