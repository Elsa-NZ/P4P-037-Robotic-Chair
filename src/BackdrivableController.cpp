#include "BackdrivableController.h"
#include "PID.h"
#include "definitions.h"
// #include "ShaftEncoder.h"
// #include "MotorEncoder.h"



PID* PIDController = nullptr;

BackdrivableController::BackdrivableController() {
    // Initialize position PID gains
    kp_pos = 30.0f;
    ki_pos = 5.0f; 
    kd_pos = 0.0f; 
    // Initialize position PID windup and saturation limits
    windupLimit_pos = 40.0f; 
    saturationHigh_pos =  0.9*255; 
    saturationLow_pos = 0.1*255; 

    // Create PID controllers for position and velocity
    PIDController = new PID(kp_pos, ki_pos, kd_pos, windupLimit_pos, saturationHigh_pos, saturationLow_pos, 0);
}

BackdrivableController::~BackdrivableController() {
    delete PIDController;
}

float BackdrivableController::update(float targetError, float currentPosition, float dt, float magneticEncoderPosition) {
    // Update position PID controller
    

    error = currentPosition - magneticEncoderPosition; // Calculate the angle error for PID

    float torqueFromChair = BAR_WEIGHT_IN_NEWTON * sin(magneticEncoderPosition * (PI / 180.0f)) * BAR_CENTROID; // Calculate torque from the bar weight based on current position
    float torqueRequired = SPRING_CONSTANT * targetError; // Calculate required torque to achieve target position
    float requiredDisplacement = (torqueRequired - torqueFromChair * (magneticEncoderPosition/abs(magneticEncoderPosition)))/SPRING_CONSTANT; // Calculate the required displacement to achieve the target position

    // if (abs(error) < 0.5 && targetError == 0) { // degrees
    //     error = 0; 
    // }

    float positionOutput = PIDController->update(requiredDisplacement, error, dt); // Note: +ve  turn CCW, -ve  turn CW (NEED TO CHECK PRIOR TESTING)

    // float feedforward = targetError/DEGREES_PER_AMP; // Feedforward term to assist in reaching target position (value from testing)

    // if (positionOutput != 0) {
    //     positionOutput += 25.5*(abs(positionOutput)/positionOutput);
    // }

    // Serial.println(positionOutput);

    // Store the final output for retrieval
    // output = positionOutput - feedforward;
    output = positionOutput;

    return output;
}

int BackdrivableController::calibrate() {
    // 1. Move to a +25 deg position
    // 2. Read the shaft encoder and magnetic encoder values
    // 3. Repeat in 5 degree increments until -45 deg is reached
    // 4. Determine the 
}

float BackdrivableController::getOutput() {
    return output;
}

float BackdrivableController::getError() {
    return error;
}