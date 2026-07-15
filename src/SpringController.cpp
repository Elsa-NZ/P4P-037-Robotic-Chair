#include "SpringController.h"
#include "PID.h"
#include "definitions.h"
// #include "ShaftEncoder.h"
// #include "MotorEncoder.h"



PID* PIDLinearController = nullptr;
PID* PIDNonlinearController = nullptr;

SpringController::SpringController() {
    // Initialize linear PID gains
    kp_linear = 4.0f; 
    kd_linear = 0.0f; 
    
    saturationHigh_linear = 0.9f * 255.0f;
    saturationLow_linear = 0.1f * 255.0f;

    // Create PID controllers for position and velocity
    PIDLinearController = new PID(kp_linear, 0, kd_linear, 0, saturationHigh_linear, saturationLow_linear, 0);

    // Initialize nonlinear PID gains
    kp_nonlinear = 0.5f;
    kd_nonlinear = 0.0f;

    // Initialize nonlinear PID windup and saturation limits
    saturationHigh_nonlinear = 0.9f * 255.0f;
    saturationLow_nonlinear = 0.1f * 255.0f;

    // Create PID controllers for nonlinear control
    PIDNonlinearController = new PID(kp_nonlinear, 0, kd_nonlinear, 0, saturationHigh_nonlinear, saturationLow_nonlinear, 0);
}

SpringController::~SpringController() {
    delete PIDNonlinearController;
    delete PIDLinearController;
}

float SpringController::updateLinear(float targetPosition, float currentPosition, float dt, float magneticEncoderPosition) {
    // Update position PID controller
    

    error = magneticEncoderPosition - targetPosition; // Calculate the angle error for PID

    float positionOutput = PIDLinearController->update(targetPosition, error, dt); // Note: +ve  turn CCW, -ve  turn CW (NEED TO CHECK PRIOR TESTING)

    // if (positionOutput != 0) {
    //     positionOutput += 25.5*(abs(positionOutput)/positionOutput);
    // }

    // Serial.println(positionOutput);

    // Store the final output for retrieval
    output = positionOutput;

    return output;
}

float SpringController::updateNonlinear(float targetPosition, float currentPosition, float dt, float magneticEncoderPosition) {
    // Update position PID controller
    

    error = magneticEncoderPosition - targetPosition; // Calculate the angle error for PID

    // Apply the nonlinear spring profile while preserving the sign of the error
    float errorNonlinear = error * fabs(error);

    float positionOutput = PIDNonlinearController->update(targetPosition, errorNonlinear, dt); // Note: +ve  turn CCW, -ve  turn CW (NEED TO CHECK PRIOR TESTING)

    // if (positionOutput != 0) {
    //     positionOutput += 25.5*(abs(positionOutput)/positionOutput);
    // }

    // Serial.println(positionOutput);

    // Store the final output for retrieval
    output = positionOutput;

    return output;
}

float SpringController::getOutput() {
    return output;
}

float SpringController::getError() {
    return error;
}