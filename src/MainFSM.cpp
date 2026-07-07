#include "MainFSM.h"
#include "OperatingFSM.h"
#include "ReadHardwareState.h"

MainFSM::MainFSM() : currentState(IDLE), timeInState(0.0f) {}

void MainFSM::updateFSM() {
  switch (currentState) {
    case IDLE:
      idle();
      // Transitions
      if (outsideSafeRange()) {
        setState(ERROR);
      } else if (readEnableButton() && (isHomed == false)) {
        setState(HOMING);
      } else if (readEnableButton() && (isHomed == true)) {
        setState(OPERATING);
      }
      break;
    case OPERATING:
      operating();
      // Transitions
      if (readEnableButton() == false) {
        setState(IDLE);
      } else if (outsideSafeRange()) {
        setState(ERROR);
      }
      break;

    case HOMING:
      homing();
      // Transition back to IDLE after homing is complete
      if (isHomed) {
        setState(IDLE);
      }
      break;

    case ERROR:
      error();
      // Transition
      if (readEnableButton() == false) {
        setState(IDLE);
      }
      break;
  }
}

void MainFSM::setState(State newState) {
  currentState = newState;
  timeInState = 0.0f;  // Reset time in state on transition
}
