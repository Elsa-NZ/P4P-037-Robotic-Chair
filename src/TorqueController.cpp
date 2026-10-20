#include "TorqueController.h"
#include "PID.h"
#include "definitions.h"



PID* torquePID = nullptr;
bool SpringActive = false;

TorqueController::TorqueController() {
    // Initialize linear PID gains (BACKDRIVE DEMO: 15.0, 1.0, 0.5, 0.5)
    kp_linear = 10.0f; 
    ki_linear = 50.0f;
    kd_linear = 0.7f; 
    kf_linear = 0.9f; // Feedforward gain for linear control
    
    saturationHigh_linear = 0.9f * 255.0f;
    saturationLow_linear = 0.0;

    // Create PID controllers for position and velocity
    torquePID = new PID(kp_linear, ki_linear, kd_linear, 0.5, saturationHigh_linear, saturationLow_linear, 0);
}

TorqueController::~TorqueController() {
    delete torquePID;
}

float TorqueController::Spring(float springConstant, float equilibriumPosition, float motorPosition, float dt, float shaftPosition) {
    // Update position PID controller
    float displacement = equilibriumPosition - shaftPosition; // Calculate the angle error for PID
    float targetTorque = displacement * springConstant; // Calculate the target compression based on the spring's compression ratio
    torquePID->setGains(kp_linear, ki_linear, kd_linear, 0.5); // Update the PID gains for linear control
    SpringActive = true; // Set the spring active flag to true
    output = updateTorqueController(motorPosition, dt, shaftPosition, targetTorque); // Update the torque controller with the new target torque
    SpringActive = false; // Set the spring active flag to false
    return output;
}

float TorqueController::SCurveTraj(float startTorque, float endTorque, float travelTime, float dt, unsigned long startTime, float motorPosition, float shaftPosition) {

    long currentTime = millis() - startTime; // Current time in milliseconds since the start of the trajectory
    float delta = endTorque - startTorque; // Define the change in position based on the S-curve trajectory

    float s = ((float)currentTime) / travelTime;
    float targetTorque;

    if (currentTime < travelTime) {
        targetTorque = startTorque + delta * (10*s*s*s - 15*s*s*s*s + 6*s*s*s*s*s);
    } else {
        targetTorque = endTorque; // stay in position after the trajectory is complete
    }

    float power = updateTorqueController(motorPosition, dt, shaftPosition, targetTorque); // Update the torque controller with the new target torque

    return power;
}

float TorqueController::updateTorqueController(float motorPosition, float dt, float shaftPosition, float targetTorque) {
    // Update position PID controller

    if (!SpringActive) {
        torquePID->setGains(kp_linear, ki_linear, kd_linear, 1); // If the spring is not active, set the PID gains to zero
    }
    
    float targetCompression, targetTorqueMinusChair, torque = 0.0f;
    float torqueFromChair = CHAIRBACK_WEIGHT_IN_NEWTON * sin(shaftPosition * (PI / 180.0f)) * 1.45 * BAR_CENTROID; // Calculate torque from the chair back weight based on current position (adjusted centriod for the bar based on exprimental testing)
    targetTorqueMinusChair = targetTorque - torqueFromChair; // Add the torque from the chair back weight to the target torque

    if (targetTorqueMinusChair < 0) {
        targetCompression = -0.0096 * pow(targetTorqueMinusChair, 2) + 0.6876 * targetTorqueMinusChair; 
    } else {
        targetCompression = 0.0096 * pow(targetTorqueMinusChair, 2) + 0.6876 * targetTorqueMinusChair; //displacementCorrection = -0.0027 * pow(torqueFromChair,2) + 0.3236 * torqueFromChair; 
    }

    float currentCompression = (motorPosition - shaftPosition); // Calculate the current compression

    float compressionOutput = torquePID->update(targetCompression, currentCompression, dt);


    float feedforward = kf_linear * (targetTorque - 0.5*torqueFromChair)/ TORQUE_CONSTANT; // Feedforward term to assist in reaching target position (kf from testing)


    if (targetCompression > 0) {
        torque = (-0.6876 + sqrt(0.6876 * 0.6876 + 4.0 * 0.0096 * targetCompression))/ (2.0 * 0.0096); // Quadratic formula to calculate the torque from the spring based on the current displacement
    } else {
        torque = (-0.6876 + sqrt(0.6876 * 0.6876 - 4.0 * 0.0096 * targetCompression))/ (2.0 * -0.0096); // Torque measured from spring
    }

    output = compressionOutput + feedforward;

    return output;
}
