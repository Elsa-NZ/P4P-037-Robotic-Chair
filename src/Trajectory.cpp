#include "Trajectory.h"
#include "PID.h"
#include "definitions.h"
#include "Motor.h"
#include "PositionController.h"

extern PositionController controller;

PID* TrajectoryControllerPosition = nullptr;

Trajectory::Trajectory() {
    // Initialize position PID gains
    kp_pos = 5.5f; 
    ki_pos = 5.5f; 
    kd_pos = 0.1f; 

    // Initialize position PID windup and saturation limits
    windupLimit_pos = 30.0f; 
    saturationHigh_pos = 0.9*255; 
    saturationLow_pos = 0.0*255; 

    TrajectoryControllerPosition = new PID(kp_pos, ki_pos, kd_pos, windupLimit_pos, saturationHigh_pos, saturationLow_pos, 0);
    }

Trajectory::~Trajectory() {
    delete TrajectoryControllerPosition;
}



float Trajectory::PosSCurveTraj(float startPos, float endPos, float travelTime, float dt, unsigned long startTime, float ShaftPosition) {
    // Implement the logic for following an S-curve trajectory

    float avgVelocity = abs((endPos - startPos) / (travelTime/1000.0)); // Define the velocity based on the S-curve trajectory
    float delta = endPos - startPos; // Define the change in position based on the S-curve trajectory

    // feedfoward gains:
    float ka_linear = 0.09f; // Feedforward gain acceleration
    float kv_linear = 0.4f; // Feedforward gain velocity
    float ks_linear = 1.25f; // Feedforward gain angle

    kp_pos = 3.5;
    ki_pos = 1.2;
    kd_pos = 0.12;

    TrajectoryControllerPosition->setGains(kp_pos, ki_pos, kd_pos, windupLimit_pos); // Update the PID gains for position control

    long currentTime = millis() - startTime; // Current time in milliseconds since the start of the trajectory

    float s = ((float)currentTime) / travelTime;
    float targetPosition, velocity, acceleration = 0.0f; // Define the target position based on the S-curve trajectory

    if (currentTime < travelTime) {
        targetPosition = startPos + delta * (10*s*s*s - 15*s*s*s*s + 6*s*s*s*s*s);
        velocity = (delta / travelTime) * (30*s*s - 60*s*s*s + 30*s*s*s*s);
        acceleration = (delta / (travelTime * travelTime)) * (60*s - 180*s*s + 120*s*s*s);
    } else {
        targetPosition = endPos; // stay in position after the trajectory is complete
        velocity = 0.0f; // No change in position, so velocity is zero
        acceleration = 0.0f; // No change in velocity, so acceleration is zero
    }


    float position = TrajectoryControllerPosition->update(targetPosition, ShaftPosition, dt);
    double feedforward = 1000000 * ka_linear * acceleration + 1000 * kv_linear * velocity - ks_linear * CHAIRBACK_WEIGHT_IN_NEWTON * sin(targetPosition * (PI / 180.0f)) * BAR_CENTROID; // Feedforward term to assist in reaching target position (value from testing)

    float power = position + feedforward;

    return power;
}
