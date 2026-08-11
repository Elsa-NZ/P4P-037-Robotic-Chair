#include "BackdrivableController.h"
#include "PID.h"
#include "definitions.h"
// #include "ShaftEncoder.h"
// #include "MotorEncoder.h"



PID* PIDController = nullptr;

BackdrivableController::BackdrivableController() {
    // Initialize position PID gains
    kp_pos = 50.0f; //53
    ki_pos = 16.0f; //16
    kd_pos = 1.0f; //1 (keep it at 1 to prevent wobbles)
    // Initialize position PID windup and saturation limits
    windupLimit_pos = 200.0f; 
    saturationHigh_pos =  0.9*255; 
    saturationLow_pos = 0.1*255; 

    filterdOutput = 0.0f;
    prevalue1 = 0.0f;
    // prevalue2 = 0.0f;
    // prevalue3 = 0.0f;
    // prevalue4 = 0.0f;
    // prevalue5 = 0.0f;

    // Create PID controllers for position and velocity
    PIDController = new PID(kp_pos, ki_pos, kd_pos, windupLimit_pos, saturationHigh_pos, saturationLow_pos, 0);
}

BackdrivableController::~BackdrivableController() {
    delete PIDController;
}

float BackdrivableController::update(float targetTorque, float motorAngle, float dt, float shaftAngle) {
    // Update position PID controller
    float torqueRequired, torqueFromChair, requiredDisplacement, torque = 0.0f;
    

    error = motorAngle - shaftAngle; // Calculate the angle error for PID
    // error = MA_filter(error); // Apply moving average filter to the error

    // Serial.print("Manetic Encoder Position: ");
    // Serial.println(magneticEncoderPosition);

    
    torqueFromChair = CHAIRBACK_WEIGHT_IN_NEWTON * sin(shaftAngle * (PI / 180.0f)) * BAR_CENTROID; // Calculate torque from the bar weight based on current position
    //float torqueRequired = pow(targetError, 3)*0.0153 + 0.1265*pow(targetError, 2)+2.1336*targetError + 1.8911; // Calculate required torque to achieve target position
    // if (targetTorque > 0) {
    //     torqueRequired = 0.1965 * pow(targetError, 2) + 2.7137*targetError; // Calculate required torque to achieve target position
    // } else {
    //     torqueRequired = -0.1849 * pow(targetError,2) + 0.4371*targetError; // Calculate required torque to achieve target position
    // }
    torque = targetTorque - torqueFromChair; // Calculate the net torque required to achieve target position
    // requiredDisplacement = -0.0001*pow(torque,3) - 0.0007*pow(torque,2) + 0.4253*torque -1.4922; // Calculate the required displacement to achieve the target position
    if (torque < 0) {
        // requiredDisplacement = 0.0027 * pow(torque,2) + 0.3236 * torque; // Calculate the required displacement to achieve the target position
        requiredDisplacement = 0.0096 * pow(torque, 2) + 0.6876 * torque;
    } else {
        // requiredDisplacement = 0.0096 * pow(torque, 2) + 0.6876 * torque; // Calculate the required displacement to achieve the target position
        requiredDisplacement = -0.0096 * pow(torque,2) + 0.6876 * torque;
    }
    
    
    // requiredDisplacement += 0.5; // Backlash in spring
    Serial.print(">Required Torque: ");
    Serial.println(torque);
    Serial.print(">Required Displacement: ");
    Serial.println(requiredDisplacement);
    Serial.print(">Displacement: ");
    Serial.println(error);

    // if (abs(error) < 0.5 && targetError == 0) { // degrees
    //     error = 0; 
    // }

    float positionOutput = PIDController->update(requiredDisplacement, error, dt); // Note: +ve  turn CCW, -ve  turn CW (NEED TO CHECK PRIOR TESTING)

    Serial.print(">Position Output: ");
    Serial.println(positionOutput);

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

float BackdrivableController::MA_filter(float value) {
    filterdOutput = (value + prevalue1) / 2.0f;
    // prevalue5 = prevalue4;
    // prevalue4 = prevalue3;
    // prevalue3 = prevalue2;
    // prevalue2 = prevalue1;
    prevalue1 = value;
    return filterdOutput;
}