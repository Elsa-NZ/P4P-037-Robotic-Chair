#include "BackdrivableController.h"
#include "PID.h"
#include "definitions.h"
// #include "ShaftEncoder.h"
// #include "MotorEncoder.h"



PID* PIDController = nullptr;

BackdrivableController::BackdrivableController() {
    // Initialize position PID gains
    kp_pos = 1.0f;
    ki_pos = 0.0f; 
    kd_pos = 0.0f; 
    // Initialize position PID windup and saturation limits
    windupLimit_pos = 60.0f; 
    saturationHigh_pos = 400.0f; 
    saturationLow_pos = -400.0f; 

    // Create PID controllers for position and velocity
    PIDController = new PID(kp_pos, ki_pos, kd_pos, windupLimit_pos, saturationHigh_pos, saturationLow_pos);
}

BackdrivableController::~BackdrivableController() {
    delete PIDController;
}

float BackdrivableController::update(float targetError, float currentPosition, float dt, float magneticEncoderPosition) {
    // Update position PID controller
    

    float error = currentPosition - magneticEncoderPosition; // Calculate the angle error for PID

    float positionOutput = PIDController->update(targetError, error, dt); // Note: +ve  turn CCW, -ve  turn CW (NEED TO CHECK PRIOR TESTING)

    // if (positionOutput != 0) {
    //     positionOutput += 25.5*(abs(positionOutput)/positionOutput);
    // }

    // Serial.println(positionOutput);

    // Store the final output for retrieval
    output = positionOutput;

    return output;
}

float BackdrivableController::getOutput() {
    return output;
}