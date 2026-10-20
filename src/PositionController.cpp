#include "PositionController.h"
#include "PID.h"
#include "definitions.h"

PID* positionPID = nullptr;

PositionController::PositionController() {
    // Initialize position PID gains
    kp_pos = 3.5;
    ki_pos = 1.2;
    kd_pos = 0.12;
    // Initialize position PID windup and saturation limits
    windupLimit_pos = 6000.0f; 
    saturationHigh_pos = 0.9*255; 
    saturationLow_pos = 0.0*255; 

    // Initialize velocity PID gains
    kp_vel = 1.5f; 
    ki_vel = 0.7f; 
    kd_vel = 0.0f; 
    // Initialize velocity PID windup and saturation limits
    windupLimit_vel = 180.0f; 
    saturationHigh_vel = 0.9*255; 
    saturationLow_vel = 0.0*255; 

    // Create PID controllers for position and velocity
    positionPID = new PID(kp_pos, ki_pos, kd_pos, windupLimit_pos, saturationHigh_pos, saturationLow_pos, 0);
}

PositionController::~PositionController() {
    delete positionPID;
}

float PositionController::updatePositionController(float targetPosition, float currentPosition, float dt, float currentFrequency, float currentPhase) {
    float positionOutput = positionPID->update(targetPosition, currentPosition, dt);

    float ka_linear = 0.09f; // Feedforward gain acceleration
    float kv_linear = 0.4f; // Feedforward gain velocity
    float ks_linear = 1.25f; // Feedforward gain angle

    double velocity = 2 * PI * currentFrequency * amplitude * cos(currentPhase); // Calculate the velocity based on the current frequency and phase
    double acceleration = amplitude * (2*PI*k*cos(currentPhase)-pow(2*PI*currentFrequency, 2)*sin(currentPhase)); // Calculate the acceleration based on the current frequency and phase
    
    double feedforward = ka_linear * acceleration + kv_linear * velocity - ks_linear * CHAIRBACK_WEIGHT_IN_NEWTON * sin(targetPosition * (PI / 180.0f)) * BAR_CENTROID; // Feedforward term to assist in reaching target position (value from testing)
    output = positionOutput + feedforward*0; // Use position output without feedfoward

    return output;
}

float PositionController::getOutput() {
    return output;
}