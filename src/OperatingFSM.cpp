#include "OperatingFSM.h"
#include "MainFSM.h"
#include "HardwareFunctions.h"
#include "CascadePosVelController.h"
#include "definitions.h"
#include "MotorEncoder.h"

extern CascadePosVelController controller; // Declare the external instance of CascadePosVelController
OperatingFSM::OperatingFSM() : currentState(IDLE), timeInState(0.0f) {}

void OperatingFSM::updateFSM() {
  switch (currentState) {
    case IDLE:
      idle();
      // Transitions
      if (readEnableButton() && (readSwitchDIP() == 0b001)) {
        setState(CONTROLLER1);
      } else if (readEnableButton() && (readSwitchDIP() == 0b010)) {
        setState(CONTROLLER2);
      } else if (readEnableButton() && (readSwitchDIP() == 0b011)) {
        setState(CONTROLLER3);
      } else if (readEnableButton() && (readSwitchDIP() == 0b100)) {
        setState(CONTROLLER4);
      } else if (readEnableButton() && (readSwitchDIP() == 0b101)) {
        setState(CONTROLLER5);
      } else if (readEnableButton() && (readSwitchDIP() == 0b110)) {
        setState(CONTROLLER6);
      } else if (readEnableButton() && (readSwitchDIP() == 0b111)) {
        setState(CONTROLLER7);
      }
      break;
    case CONTROLLER1:
      controller1();
      // Transitions
      if (!readEnableButton() || (readSwitchDIP() == 0b000)) {
        setState(IDLE);
      }
      break;
    case CONTROLLER2:
      controller2();
      // Transitions
      if (!readEnableButton() || (readSwitchDIP() == 0b000)) {
        setState(IDLE);
    }
      break;
    case CONTROLLER3:
      controller3();
      // Transitions
      if (!readEnableButton() || (readSwitchDIP() == 0b000)) {
        setState(IDLE);
      }
      break;
    case CONTROLLER4:
        controller4();
        // Transitions
        if (!readEnableButton() || (readSwitchDIP() == 0b000)) {
            setState(IDLE);
        }
        break;
    case CONTROLLER5:
        controller5();
        // Transitions
        if (!readEnableButton() || (readSwitchDIP() == 0b000)) {
            setState(IDLE);
        }
        break;
    case CONTROLLER6:
        controller6();
        // Transitions
        if (!readEnableButton() || (readSwitchDIP() == 0b000)) {
            setState(IDLE);
        }
        break;
    case CONTROLLER7:
        controller7();
        // Transitions
        if (!readEnableButton() || (readSwitchDIP() == 0b000)) {
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
  if (millis() % 10 == 0) {
    float dt = 0.01f; // 10 ms in seconds
    float targetPosition = 0.0f; // Define your target position here
    float motorPosition = (pulses/(2000.0*51.0))*360.0; // Current position from encoder(); // Read the current position from the shaft
    float power = controller.update(targetPosition, motorPosition, dt);

     if (power < 0) {
      digitalWrite(DIRECTION_PIN, HIGH); // CW
      power = abs(power);
    } else {
      digitalWrite(DIRECTION_PIN, LOW); // CCW
    }

    // Use the output to control the motor (set PWM value)
    analogWrite(PWM_PIN, power);
  }
  // Serial.println("OperatingFSM: CONTROLLER1 state active");
}

void OperatingFSM::controller2() {
  // Implement the behavior for CONTROLLER2 state
  Serial.println("OperatingFSM: CONTROLLER2 state active");
}

void OperatingFSM::controller3() {
  // Implement the behavior for CONTROLLER3 state
  Serial.println("OperatingFSM: CONTROLLER3 state active");
}

void OperatingFSM::controller4() {
  // Implement the behavior for CONTROLLER4 state
  Serial.println("OperatingFSM: CONTROLLER4 state active");
}

void OperatingFSM::controller5() {
  // Implement the behavior for CONTROLLER5 state
  Serial.println("OperatingFSM: CONTROLLER5 state active");
}

void OperatingFSM::controller6() {
  // Implement the behavior for CONTROLLER6 state
  Serial.println("OperatingFSM: CONTROLLER6 state active");
}

void OperatingFSM::controller7() {
  // Implement the behavior for CONTROLLER7 state
  Serial.println("OperatingFSM: CONTROLLER7 state active");
}

