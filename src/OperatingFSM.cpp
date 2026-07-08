#include "OperatingFSM.h"
#include "MainFSM.h"
#include "HardwareFunctions.h"

OperatingFSM::OperatingFSM() : currentState(IDLE), timeInState(0.0f) {}

void OperatingFSM::updateFSM() {
  switch (currentState) {
    case IDLE:
      idle();
      // Transitions
      if (readEnableButton()) {
        setState(CONTROLLER1);
      }
      break;
    case CONTROLLER1:
      controller1();
      // Transitions
      if (!readEnableButton()) {
        setState(IDLE);
      }
      break;
    case CONTROLLER2:
      controller2();
      // Transitions
      if (!readEnableButton()) {
        setState(IDLE);
    }
      break;
    case CONTROLLER3:
      controller3();
      // Transitions
      if (!readEnableButton()) {
        setState(IDLE);
      }
      break;
    case CONTROLLER4:
        controller4();
        // Transitions
        if (!readEnableButton()) {
            setState(IDLE);
        }
        break;
    case CONTROLLER5:
        controller5();
        // Transitions
        if (!readEnableButton()) {
            setState(IDLE);
        }
        break;
    case CONTROLLER6:
        controller6();
        // Transitions
        if (!readEnableButton()) {
            setState(IDLE);
        }
        break;
    case CONTROLLER7:
        controller7();
        // Transitions
        if (!readEnableButton()) {
            setState(IDLE);
        }
  }
}

void OperatingFSM::setState(State newState) {
  currentState = newState;
  timeInState = 0.0f;  // Reset time in state on transition
}

void OperatingFSM::idle() {
  // Implement the behavior for the IDLE state
}

void OperatingFSM::controller1() {
  // Implement the behavior for CONTROLLER1 state
}

void OperatingFSM::controller2() {
  // Implement the behavior for CONTROLLER2 state
}

void OperatingFSM::controller3() {
  // Implement the behavior for CONTROLLER3 state
}

void OperatingFSM::controller4() {
  // Implement the behavior for CONTROLLER4 state
}

void OperatingFSM::controller5() {
  // Implement the behavior for CONTROLLER5 state
}

void OperatingFSM::controller6() {
  // Implement the behavior for CONTROLLER6 state
}

void OperatingFSM::controller7() {
  // Implement the behavior for CONTROLLER7 state
}

