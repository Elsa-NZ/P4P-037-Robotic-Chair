#include "SpringController.h"
#include "PID.h"
#include "definitions.h"
// #include "ShaftEncoder.h"
// #include "MotorEncoder.h"



PID* PIDLinearController = nullptr;
PID* PIDNonlinearController = nullptr;

SpringController::SpringController() {
    // Initialize linear PID gains
    kp_linear = 10.0f;
    ki_linear = 5.0f;
    kd_linear = 0.0f; 
    kf_linear = 0.8f; // Feedforward gain for linear control
    
    saturationHigh_linear = 0.9f * 255.0f;
    saturationLow_linear = 0.0;

    // Create PID controllers for position and velocity
    PIDLinearController = new PID(kp_linear, ki_linear, kd_linear, 0.5, saturationHigh_linear, saturationLow_linear, 0);

    // Initialize nonlinear PID gains
    kp_nonlinear = 0.5f;
    ki_nonlinear = 0.0f;
    kd_nonlinear = 0.0f;

    // Initialize nonlinear PID windup and saturation limits
    saturationHigh_nonlinear = 0.9f * 255.0f;
    saturationLow_nonlinear = 0.1f * 255.0f;

    // Create PID controllers for nonlinear control
    PIDNonlinearController = new PID(kp_nonlinear, ki_nonlinear, kd_nonlinear, 0, saturationHigh_nonlinear, saturationLow_nonlinear, 0);
}

SpringController::~SpringController() {
    delete PIDNonlinearController;
    delete PIDLinearController;
}

float SpringController::updateLinear(float equilibriumPosition, float motorPosition, float dt, float shaftPosition) {
    // Update position PID controller

    float displacement = equilibriumPosition - shaftPosition; // Calculate the angle error for PID
    
    float targetCompression = displacement / COMPRESSION_RATIO; // Calculate the target compression based on the spring's compression ratio

    float currentCompression = (motorPosition - shaftPosition); // Calculate the current compression

    float compressionOutput = PIDLinearController->update(targetCompression, currentCompression, dt);

    float feedforward = kf_linear * (targetCompression / DEGREES_PER_AMP) * (229.5/4); // Feedforward term to assist in reaching target position (value from testing)

    if (targetCompression < 0) {
        feedforward = feedforward - 25.5;
    } else {
        feedforward = feedforward + 25.5;
    }

    output = compressionOutput + feedforward;

    if (output > saturationHigh_linear) {
        output = saturationHigh_linear;
    } else if (output < saturationLow_linear && output > 0) {
        output = saturationLow_linear;
    } else if (output > -saturationLow_linear && output < 0) {
        output = -saturationLow_linear;
    } else if (output < -saturationHigh_linear) {
        output = -saturationHigh_linear;
    }

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