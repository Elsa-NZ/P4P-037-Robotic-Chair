#include "OperatingFSM.h"
#include "MainFSM.h"
#include "HardwareFunctions.h"
#include "PositionController.h"
#include "definitions.h"
#include "MotorEncoder.h"
#include "TorqueController.h"
#include "Motor.h"
#include "Trajectory.h"

extern PositionController controller; // Declare the external instance of CascadePosVelController
OperatingFSM::OperatingFSM() : currentState(IDLE), switchStateTimer(0.0f) {}
extern TorqueController springController; // Declare the external instance of SpringController
extern Trajectory trajectory; // Declare the external instance of Trajectory

void OperatingFSM::updateFSM() {
  switch (currentState) {
    case IDLE:
      idle();
      starttime = millis(); // Reset the start time when entering the IDLE state
      if (readEnableButton() && (readSwitchDIP() == 0b000)) {
        switchStateTimer = millis(); // Reset the switch state timer
      }
      // Transitions
      if (readEnableButton() && (readSwitchDIP() == 0b001) && (millis() - switchStateTimer >= 2000)) { // Check if the DIP switch is set to 001 and the button has been held for at least 1 second
        setState(CONTROLLER1);
        lastUpdateTime = millis(); // Initialize lastUpdateTime when entering CONTROLLER1
      } else if (readEnableButton() && (readSwitchDIP() == 0b010) && (millis() - switchStateTimer >= 2000)) {
        setState(CONTROLLER2);
        lastUpdateTime = millis(); // Initialize lastUpdateTime when entering CONTROLLER2
      } else if (readEnableButton() && (readSwitchDIP() == 0b011) && (millis() - switchStateTimer >= 2000)) {
        setState(CONTROLLER3);
        lastUpdateTime = millis(); // Initialize lastUpdateTime when entering CONTROLLER3
      } else if (readEnableButton() && (readSwitchDIP() == 0b100) && (millis() - switchStateTimer >= 2000)) {
        setState(CONTROLLER4);
        lastUpdateTime = millis(); // Initialize lastUpdateTime when entering CONTROLLER4
      } else if (readEnableButton() && (readSwitchDIP() == 0b101) && (millis() - switchStateTimer >= 2000)) {
        setState(CONTROLLER5);
        lastUpdateTime = millis(); // Initialize lastUpdateTime when entering CONTROLLER5
      } else if (readEnableButton() && (readSwitchDIP() == 0b110) && (millis() - switchStateTimer >= 2000)) {
        setState(CONTROLLER6);
        lastUpdateTime = millis(); // Initialize lastUpdateTime when entering CONTROLLER6
      } else if (readEnableButton() && (readSwitchDIP() == 0b111) && (millis() - switchStateTimer >= 2000)) {
        setState(CONTROLLER7);
        lastUpdateTime = millis(); // Initialize lastUpdateTime when entering CONTROLLER7
      }
      break;
    case CONTROLLER1:
      controller1();
      // Transitions
      if (!readEnableButton() || (readSwitchDIP() == 0b000)) {
        setState(IDLE);
        switchStateTimer = millis();  // Reset the switch state timer on transition
      }
      break;
    case CONTROLLER2:
      controller2();
      // Transitions
      if (!readEnableButton() || (readSwitchDIP() == 0b000)) {
        setState(IDLE);
        switchStateTimer = millis();  // Reset the switch state timer on transition
    }
      break;
    case CONTROLLER3:
      controller3();
      // Transitions
      if (!readEnableButton() || (readSwitchDIP() == 0b000)) {
        setState(IDLE);
        switchStateTimer = millis();  // Reset the switch state timer on transition
      }
      break;
    case CONTROLLER4:
        controller4();
        // Transitions
        if (!readEnableButton() || (readSwitchDIP() == 0b000)) {
            setState(IDLE);
            switchStateTimer = millis();  // Reset the switch state timer on transition
        }
        break;
    case CONTROLLER5:
        controller5();
        // Transitions
        if (!readEnableButton() || (readSwitchDIP() == 0b000)) {
            setState(IDLE);
            switchStateTimer = millis();  // Reset the switch state timer on transition
        }
        break;
    case CONTROLLER6:
        controller6();
        // Transitions
        if (!readEnableButton() || (readSwitchDIP() == 0b000)) {
            setState(IDLE);
            switchStateTimer = millis();  // Reset the switch state timer on transition
        }
        break;
    case CONTROLLER7:
        controller7();
        // Transitions
        if (!readEnableButton() || (readSwitchDIP() == 0b000)) {
            setState(IDLE);
            switchStateTimer = millis();  // Reset the switch state timer on transition
        }
  }
}

void OperatingFSM::setState(State newState) {
  currentState = newState;
  switchStateTimer = millis();  // Reset the switch state timer on transition
}

void OperatingFSM::idle() {
  // Implement the behavior for the IDLE state
  move(0); // Stop the motor
}

void OperatingFSM::controller1() { // SPRING CONTROLLER
  // Implement the behavior for CONTROLLER1 state
  unsigned long dt = getdt(); // Get the time difference since the last update
    float motorPosition = (pulses/(2000.0*51.0))*360.0; // Current position from encoder(); // Read the current position from the shaft
    float shaftPosition = (readShaftEncoderADC() - zeroShaftPosition) * MAGNETIC_ENCODER_RESOLUTION;
    // MAKE SURE U UPDATE THIS !!
    float power = springController.Spring(0.15, 0, motorPosition, dt, shaftPosition);

    move(power);

}

void OperatingFSM::controller2() { // BACKDRIVABLE CONTROLLER
  // Implement the behavior for CONTROLLER2 state

    unsigned long dt = getdt(); // Get the time difference since the last update
    float motorPosition = (pulses/(2000.0*51.0))*360.0; // Current position from encoder(); // Read the current position from the shaft
    float shaftPosition = (readShaftEncoderADC() - zeroShaftPosition) * MAGNETIC_ENCODER_RESOLUTION;
    // MAKE SURE U UPDATE THIS !!
    float power = springController.updateTorqueController(motorPosition, dt, shaftPosition, 0.0);
    move(power);
}

void OperatingFSM::controller3() { // Position TrajectoryController
    unsigned long dt = getdt(); // Get the time difference since the last update
    float shaftPosition = (readShaftEncoderADC() - zeroShaftPosition) * MAGNETIC_ENCODER_RESOLUTION;
    // MAKE SURE U UPDATE THIS !!
    float power;
    if (millis() - starttime < 12000) {
        power = trajectory.PosSCurveTraj(0.0f, -30.0f, 10000.0f, dt, starttime, shaftPosition); 
      } else if (millis() - starttime >= 12000 && millis() - starttime < 17000) {
        power = trajectory.PosSCurveTraj(-30.0f, -5.0f, 3000.0f, dt, starttime+12000, shaftPosition);
      } else if (millis() - starttime >= 17000 && millis() - starttime < 20000) {
        power = trajectory.PosSCurveTraj(-5.0f, 20.0f, 1000.0f, dt, starttime+17000, shaftPosition);
      } else if (millis() - starttime >= 20000) {
        power = trajectory.PosSCurveTraj(20.0f, 0.0f, 2000.0f, dt, starttime+20000, shaftPosition);
      }
    move(power);
}

void OperatingFSM::controller4() { // Torque Controller
    unsigned long dt = getdt(); // Get the time difference since the last update
    float motorPosition = (pulses/(2000.0*51.0))*360.0; // Current position from encoder(); // Read the current position from the shaft
    float shaftPosition = (readShaftEncoderADC() - zeroShaftPosition) * MAGNETIC_ENCODER_RESOLUTION;
    // MAKE SURE U UPDATE THIS !!
    float power = springController.updateTorqueController(motorPosition, dt, shaftPosition, 4.0);
    move(power);
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

unsigned long OperatingFSM::getdt() {
  unsigned long currentTime = millis();
  unsigned long dt = currentTime - lastUpdateTime;
  lastUpdateTime = currentTime;
  dt = dt / 1000.0; // Convert milliseconds to seconds
  return dt;
}
