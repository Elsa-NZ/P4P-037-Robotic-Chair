#include "CascadePosVelController.h"
#include "PID.h"
#include "definitions.h"

PID* positionPID = nullptr;
PID* velocityPID = nullptr;

CascadePosVelController::CascadePosVelController() {
    // Initialize position PID gains
    kp_pos = 3.0f; // 1.5
    ki_pos = 2.0f; // 0.1
    kd_pos = 0.1f; // 0.2
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
    velocityPID = new PID(kp_vel, ki_vel, kd_vel, windupLimit_vel, saturationHigh_vel, saturationLow_vel, 0);
}

CascadePosVelController::~CascadePosVelController() {
    delete positionPID;
    delete velocityPID;
}

float CascadePosVelController::update(float targetPosition, float currentPosition, float dt, float currentFrequency, float currentPhase) {
    // Serial.print("Target Position: ");
    // Serial.println(targetPosition);
    // Serial.print("Current Position: ");
    // Serial.println(currentPosition);
    
    // Update position PID controller
    // targetPosition = targetPosition - (marginalOffset*(targetPosition/abs(targetPosition))); // Apply the marginal offset to the target position to account for inertia
    float positionOutput = positionPID->update(targetPosition, currentPosition, dt);

    // if (positionOutput != 0) {
    //     positionOutput += 25.5*(abs(positionOutput)/positionOutput);
    // }

    // int speed = 1.8310546875*analogRead(SPEED_PIN)-740; //(1.8310546875*measuredVelocity)-740)

    // Serial.println(positionOutput);

    // Update velocity PID controller with the output from the position PID as the target
    // float velocityOutput = velocityPID->update(positionOutput, 0.1*speed, dt);

    // Serial.println(velocityOutput);

    // Store the final output for retrieval
    // output = velocityOutput;
    float ka_linear = 0.1f; // Feedforward gain acceleration
    float kv_linear = 0.5f; // Feedforward gain velocity
    float ks_linear = 1.25f; // Feedforward gain angle

    double velocity = 2 * PI * currentFrequency * amplitude * cos(currentPhase); // Calculate the velocity based on the current frequency and phase
    double acceleration = amplitude * (2*PI*k*cos(currentPhase)-pow(2*PI*currentFrequency, 2)*sin(currentPhase)); // Calculate the acceleration based on the current frequency and phase
    
    Serial.print(">Target Velocity: ");
    Serial.println(velocity);
    Serial.print(">Target Acceleration: ");
    Serial.println(acceleration);


    // double feedforward = ka_linear * acceleration + kv_linear * velocity + ks_linear * (velocity/abs(velocity)) - sin(PI*targetPosition/180.0)*15.0; // Feedforward term to assist in reaching target position (value from testing)
    double feedforward = ka_linear * acceleration + kv_linear * velocity - ks_linear * CHAIRBACK_WEIGHT_IN_NEWTON * sin(targetPosition * (PI / 180.0f)) * BAR_CENTROID; // Feedforward term to assist in reaching target position (value from testing)

    // account for deadzone of motor at pwm values below 25.5 (10% of 255)
    // if (positionOutput > 0) {
    //     positionOutput += 22.5;
    // } else if (positionOutput < 0) {
    //     positionOutput -= 22.5;
    // }

    Serial.print(">Feedforward: ");
    Serial.println(feedforward);
    // float feedforward = ka_linear * 
    output = positionOutput + feedforward; // Use position output without feedfoward

    return output;
}

float CascadePosVelController::getOutput() {
    return output;
}