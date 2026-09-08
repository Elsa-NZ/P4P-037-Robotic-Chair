#include "Trajectory.h"
#include "PID.h"
#include "definitions.h"
#include "Motor.h"
#include "CascadePosVelController.h"
// #include "ShaftEncoder.h"
// #include "MotorEncoder.h"

extern CascadePosVelController controller;

PID* TrajectoryControllerPosition = nullptr;
PID* TrajectoryControllerVelocity = nullptr;

Trajectory::Trajectory() {
    // Initialize position PID gains
    kp_pos = 5.5f; 
    ki_pos = 2.5f; 
    kd_pos = 0.1f; 
    // Initialize position PID windup and saturation limits
    windupLimit_pos = 6000.0f; 
    saturationHigh_pos = 0.9*255; 
    saturationLow_pos = 0.0*255; 

    // Initialize velocity PID gains
    kp_vel = 0.9f;
    ki_vel = 0.15f;
    kd_vel = 0.0f;

    // Initialize velocity PID windup and saturation limits
    windupLimit_vel = 80.0f;
    saturationHigh_vel = 0.9*255;
    saturationLow_vel = 0.1*255;

    // Create PID controllers for position and velocity
    TrajectoryControllerPosition = new PID(kp_pos, ki_pos, kd_pos, windupLimit_pos, saturationHigh_pos, saturationLow_pos, 0);
    // TrajectoryControllerVelocity = new PID(kp_vel, ki_vel, kd_vel, windupLimit_vel, saturationHigh_vel, saturationLow_vel, 0);
}

Trajectory::~Trajectory() {
    delete TrajectoryControllerPosition;
    delete TrajectoryControllerVelocity;
}

float Trajectory::followCurveTrajectory(float shaftPosition, float dt, unsigned long time) {
    // Implement the logic for following a curve trajectory
    // This is a placeholder implementation and should be replaced with actual curve-following logic
    float targetPosition, velocity, s, delta = 0.0f; // Define the target position based on the curve trajectory
    double acceleration = 0.0f; // Define the acceleration based on the curve trajectory
    float currentPosition = shaftPosition; // Current position from shaft encoder
    long currentTime = millis() - time; // Current time in milliseconds since the start of the trajectory

    float ka_linear = 0.1f; // Feedforward gain acceleration
    float kv_linear = 0.5f; // Feedforward gain velocity
    float ks_linear = 1.25f; // Feedforward gain angle



    if (currentTime < 1500) {
        s = (float)currentTime / 1500.0f;
        targetPosition = 25.0f * (10*s*s*s - 15*s*s*s*s + 6*s*s*s*s*s);
        velocity = (25.0f / 1500.0f) * (30*s*s - 60*s*s*s + 30*s*s*s*s);
        acceleration = (25.0f / (1500.0f * 1500.0f)) * (60*s - 180*s*s + 120*s*s*s);
    } else if (currentTime >= 1500 && currentTime < 3500) {
        targetPosition = 25.0f; // stay in position for 2 seconds
        velocity = 0.0f; // No change in position, so velocity is zero
        acceleration = 0.0f; // No change in velocity, so acceleration is zero
    } else if (currentTime >= 3500 && currentTime < 5500) {
        delta = -45.0f - 25.0f;
        s = ((float)currentTime - 3500.0f) / 2000.0f;
        targetPosition = 25.0f + delta * (10*s*s*s - 15*s*s*s*s + 6*s*s*s*s*s);
        velocity = (delta / 2000.0f) * (30*s*s - 60*s*s*s + 30*s*s*s*s);
        acceleration = (delta / (2000.0f * 2000.0f)) * (60*s - 180*s*s + 120*s*s*s);
    } else if (currentTime >= 5500 && currentTime < 7500) {
        targetPosition = -45.0f; // stay in position for 2 seconds
        velocity = 0.0f; // No change in position, so velocity is zero
        acceleration = 0.0f; // No change in velocity, so acceleration is zero
    } else if (currentTime >= 7500 && currentTime < 10500) {
        delta = 45.0f; // final - initial = 0 - (-45)
        s = ((float)currentTime - 7500.0f) / 3000.0f;
        targetPosition = -45.0f + delta * (10*s*s*s - 15*s*s*s*s + 6*s*s*s*s*s);
        velocity = (delta / 3000.0f) * (30*s*s - 60*s*s*s + 30*s*s*s*s);
        acceleration = (delta / (3000.0f * 3000.0f)) * (60*s - 180*s*s + 120*s*s*s);
    } else if (currentTime >= 10500) {
        targetPosition = 0.0f; // Return to initial position
        velocity = 0.0f; // No change in position, so velocity is zero
        acceleration = 0.0f; // No change in velocity, so acceleration is zero
    }

    // Update position PID controller
    float position = TrajectoryControllerPosition->update(targetPosition, currentPosition, dt);
    double feedforward = ka_linear * acceleration + kv_linear * velocity - ks_linear * CHAIRBACK_WEIGHT_IN_NEWTON * sin(targetPosition * (PI / 180.0f)) * BAR_CENTROID; // Feedforward term to assist in reaching target position (value from testing)
    // double feedforward = 0;

    Serial.print(">Target Position: ");
    Serial.println(targetPosition);
    Serial.print(">target Velocity: ");
    Serial.println(velocity);
    Serial.print(">target Acceleration: ");
    Serial.println(acceleration);
    //print data every 100ms for analysis
    
    if (currentTime - lastTime >= 100) {
        // lastTime = currentTime;
        // Serial.print(currentTime);
        // Serial.print(", ");
        // Serial.print(targetPosition);
        // Serial.print(", ");
        // // Serial.print(">Current Position: ");
        // Serial.println(currentPosition);
    }
    
    float power = position + feedforward;

    return power;

    
}

float Trajectory::followRampTrajectory(float shaftPosition, float dt, unsigned long time) {
    // make motor follow a set ramp trajectory from 0 to +25 degree over 3 seconds
    float targetPosition = 0.0f;
    float currentPosition = shaftPosition; // Current position from shaft encoder
    long currentTime = millis() - time; // Current time in milliseconds since the start of the trajectory

    if (currentTime < 3000) {
        targetPosition = (25.0f / 3000.0f) * currentTime; // Linear ramp from 0 to 25 degrees over 3 seconds

    } else if (currentTime >= 3000 && currentTime < 5000) {
        targetPosition = 25.0f; // stay in position for 3 seconds

    } else if (currentTime >= 5000 && currentTime < 10000) {
        targetPosition = 25.0f - (70.0f / 5000.0f) * (currentTime - 5000); // Linear ramp from 25 to -45 degrees over 5 seconds
        

    } else if (currentTime >= 10000 && currentTime < 12000) {
        targetPosition = -45.0f; // stay in position for 2 seconds
        

    }else if (currentTime >= 12000 && currentTime < 15000) {
        targetPosition = -45.0f + (45.0f / 3000.0f) * (currentTime - 12000); // Linear ramp from -45 to 25 degrees over 3 seconds
        

    } else if (currentTime >= 15000) {
         targetPosition = 0.0f; // Linear ramp from -45 to 25 degrees over 3 seconds
        
    }

    // float feedfoward = 10*targetPosition; // Feedforward term to assist in reaching target position (value from testing)

    // float power = controller.update(targetPosition, currentPosition, dt);

    float power = TrajectoryControllerPosition->update(targetPosition, currentPosition, dt);

    // Serial.print(">Target Position: ");
    //print data every 100ms for analysis
    
    if (currentTime - lastTime >= 100) {
        lastTime = currentTime;
        Serial.print(currentTime);
        Serial.print(", ");
        Serial.print(targetPosition);
        Serial.print(", ");
        // Serial.print(">Current Position: ");
        Serial.println(currentPosition);
    }
    
    return power;
}

