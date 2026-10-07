#include "SpringController.h"
#include "PID.h"
#include "definitions.h"
// #include "ShaftEncoder.h"
// #include "MotorEncoder.h"



PID* PIDLinearController = nullptr;
PID* PIDNonlinearController = nullptr;

SpringController::SpringController() {
    // Initialize linear PID gains (BACKDRIVE DEMO: 15.0, 1.0, 0.5, 0.5)
    kp_linear = 25.0f; 
    ki_linear = 10.0f;
    kd_linear = 0.5f; 
    kf_linear = 0.9f; // Feedforward gain for linear control
    
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
    saturationLow_nonlinear = 0.0f * 255.0f;

    // Create PID controllers for nonlinear control
    PIDNonlinearController = new PID(kp_nonlinear, ki_nonlinear, kd_nonlinear, 0, saturationHigh_nonlinear, saturationLow_nonlinear, 0);
}

SpringController::~SpringController() {
    delete PIDNonlinearController;
    delete PIDLinearController;
}

float SpringController::updateLinear(float springConstant, float equilibriumPosition, float motorPosition, float dt, float shaftPosition) {
    // Update position PID controller
    float currenttorque, torque, displacementCorrection = 0.0f;
    float displacement = equilibriumPosition - shaftPosition; // Calculate the angle error for PID

    
    float targetTorque = displacement * springConstant; // Calculate the target compression based on the spring's compression ratio
    output = torqueController(motorPosition, dt, shaftPosition, targetTorque); // Update the torque controller with the new target torque
    

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

float SpringController::SCurveTraj(float springConstant, float startTorque, float endTorque, float travelTime, float dt, unsigned long startTime, float motorPosition, float shaftPosition) {
    // Implement the logic for following an S-curve trajectory
    // This is a placeholder implementation and should be replaced with actual S-curve-following logic

    long currentTime = millis() - startTime; // Current time in milliseconds since the start of the trajectory
    float delta = endTorque - startTorque; // Define the change in position based on the S-curve trajectory

    float s = ((float)currentTime) / travelTime;
    float targetTorque;

    if (currentTime < travelTime) {
        targetTorque = startTorque + delta * (10*s*s*s - 15*s*s*s*s + 6*s*s*s*s*s);
    } else {
        targetTorque = endTorque; // stay in position after the trajectory is complete
    }

    Serial.print(">Target Torque: ");
    Serial.println(targetTorque);

    float power = torqueController(motorPosition, dt, shaftPosition, targetTorque); // Update the torque controller with the new target torque

    return power;
}

float SpringController::torqueController(float motorPosition, float dt, float shaftPosition, float targetTorque) {
    // Update position PID controller
    
    float targetCompression, targetTorqueMinusChair, torque = 0.0f;
    float torqueFromChair = CHAIRBACK_WEIGHT_IN_NEWTON * sin(shaftPosition * (PI / 180.0f)) * 1.45 * BAR_CENTROID; // Calculate torque from the chair back weight based on current position (adjusted centriod for the bar based on exprimental testing)
    targetTorqueMinusChair = targetTorque - torqueFromChair; // Add the torque from the chair back weight to the target torque

    if (targetTorqueMinusChair < 0) {
        targetCompression = -0.0096 * pow(targetTorqueMinusChair, 2) + 0.6876 * targetTorqueMinusChair; 
    } else {
        targetCompression = 0.0096 * pow(targetTorqueMinusChair, 2) + 0.6876 * targetTorqueMinusChair; //displacementCorrection = -0.0027 * pow(torqueFromChair,2) + 0.3236 * torqueFromChair; 
    }

    float currentCompression = (motorPosition - shaftPosition); // Calculate the current compression

    float compressionOutput = PIDLinearController->update(targetCompression, currentCompression, dt);




    float feedforward = kf_linear * (targetTorque - 0.5*torqueFromChair)/ TORQUE_CONSTANT; // Feedforward term to assist in reaching target position (kf from testing)

    // if (feedforward < 0) {
    //     feedforward = feedforward - 0.1*255; 
    // } else {
    //     feedforward = feedforward + 0.1*255;
    // }

    if (targetCompression > 0) {
        torque = (-0.6876 + sqrt(0.6876 * 0.6876 + 4.0 * 0.0096 * targetCompression))/ (2.0 * 0.0096); // Quadratic formula to calculate the torque from the spring based on the current displacement
    } else {
        torque = (-0.6876 + sqrt(0.6876 * 0.6876 - 4.0 * 0.0096 * targetCompression))/ (2.0 * -0.0096); // Torque measured from spring
    }

    // if (targetCompression < 0) {
    //     feedforward = feedforward - 25.5;
    // } else {
    //     feedforward = feedforward + 25.5;
    // }

    output = compressionOutput + feedforward;

    // if (output > saturationHigh_linear) {
    //     output = saturationHigh_linear;
    // } else if (output < saturationLow_linear && output > 0) {
    //     output = saturationLow_linear;
    // } else if (output > -saturationLow_linear && output < 0) {
    //     output = -saturationLow_linear;
    // } else if (output < -saturationHigh_linear) {
    //     output = -saturationHigh_linear;
    // }

    Serial.print(">Target Compression: ");
    Serial.println(targetCompression);
    Serial.print(">Current Compression: ");
    Serial.println(currentCompression);
    // Serial.print(">Torque Before: ");
    // Serial.println(targetTorque + torqueFromChair);
    // Serial.print(">Torque After: ");
    // Serial.println(torque+torqueFromChair);


    // CODE FOR DATA
    // if (currentCompression < 0) {
    //     currenttorque = (-0.6876 + sqrt(0.6876 * 0.6876 + 4.0 * 0.0096 * currentCompression))/ (2.0 * 0.0096); // Torque measured from spring
    //     torque = (-0.6876 + sqrt(0.6876 * 0.6876 + 4.0 * 0.0096 * targetCompression))/ (2.0 * 0.0096); // Torque measured from spring
    // } else {
    //     currenttorque = (-0.6876 + sqrt(0.6876 * 0.6876 - 4.0 * 0.0096 * currentCompression))/ (2.0 * -0.0096); // Torque measured from spring
    //     torque = (-0.6876 + sqrt(0.6876 * 0.6876 - 4.0 * 0.0096 * targetCompression))/ (2.0 * -0.0096); // Torque measured from spring
    // }

    // if (millis() % 100 == 0) {
        // Serial.print(millis());
        // Serial.print(", ");
        // Serial.print(shaftPosition);
        // Serial.print(", ");
        // Serial.print(currenttorque); // Torque measured from spring
        // Serial.print(", ");
        // Serial.println(torque);
    // }
    

    return output;
}


float SpringController::getOutput() {
    return output;
}

float SpringController::getError() {
    return error;
}