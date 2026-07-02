#include "CascadePosVelController.h"
#include "PID.h"
#include "definitions.h"

PID* positionPID = nullptr;
PID* velocityPID = nullptr;

CascadePosVelController::CascadePosVelController() {
    // Initialize position PID gains
    kp_pos = 5.0f;
    ki_pos = 0.3f; 
    kd_pos = 0.1f; 
    // Initialize position PID windup and saturation limits
    windupLimit_pos = 60.0f; 
    saturationHigh_pos = 400.0f; 
    saturationLow_pos = -400.0f; 

    // Initialize velocity PID gains
    kp_vel = 0.9f; 
    ki_vel = 0.15f; 
    kd_vel = 0.0f; 
    // Initialize velocity PID windup and saturation limits
    windupLimit_vel = 80.0f; 
    saturationHigh_vel = 0.9*255; 
    saturationLow_vel = 0.1*255; 

    // Create PID controllers for position and velocity
    positionPID = new PID(kp_pos, ki_pos, kd_pos, windupLimit_pos, saturationHigh_pos, saturationLow_pos);
    velocityPID = new PID(kp_vel, ki_vel, kd_vel, windupLimit_vel, saturationHigh_vel, saturationLow_vel);
}

CascadePosVelController::~CascadePosVelController() {
    delete positionPID;
    delete velocityPID;
}

float CascadePosVelController::update(float targetPosition, float currentPosition, float dt) {
    // Update position PID controller
    float positionOutput = positionPID->update(targetPosition, currentPosition, dt);

    if (positionOutput != 0) {
        positionOutput += 25.5*(abs(positionOutput)/positionOutput);
    }

    int speed = 1.8310546875*analogRead(SPEED_PIN)-740; //(1.8310546875*measuredVelocity)-740)

    // Serial.println(positionOutput);

    // Update velocity PID controller with the output from the position PID as the target
    float velocityOutput = velocityPID->update(positionOutput, 0.1*speed, dt);

    // Serial.println(velocityOutput);

    // Store the final output for retrieval
    output = velocityOutput;

    return output;
}

float CascadePosVelController::getOutput() {
    return output;
}