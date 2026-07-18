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
    kp_pos = 10.0f; 
    ki_pos = 0.0f; 
    kd_pos = 0.7f; 
    // Initialize position PID windup and saturation limits
    windupLimit_pos = 50.0f; 
    saturationHigh_pos =  0.9*255; 
    saturationLow_pos = 0.1*255; 

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
    float targetPosition = 0.0f; // Define the target position based on the curve trajectory
    float currentPosition = shaftPosition; // Current position from shaft encoder
    long currentTime = millis() - time; // Current time in milliseconds since the start of the trajectory

    if (currentTime < 3000) {
        targetPosition = (10 * pow((float)currentTime / 3000.0f, 3)- 15 * pow((float)currentTime / 3000.0f, 4) + 6 * pow((float)currentTime / 3000.0f, 5)) * 25.0f; // Example curve logic
    } else if (currentTime >= 3000 && currentTime < 5000) {
        targetPosition = 25.0f; // stay in position for 2 seconds
    } else if (currentTime >= 5000 && currentTime < 10000) {
        targetPosition = 25+ (10 * pow(((float)currentTime-5000) / 5000.0f, 3)- 15 * pow(((float)currentTime-5000) / 5000.0f, 4) + 6 * pow(((float)currentTime-5000) / 5000.0f, 5)) * (-45-25.0f); // Example curve logic
    } else if (currentTime >= 10000 && currentTime < 12000) {
        targetPosition = -45.0f; // stay in position for 2 seconds
    } else if (currentTime >= 12000 && currentTime < 15000) {
        targetPosition = -45 + (10 * pow(((float)currentTime-12000) / 3000.0f, 3)- 15 * pow(((float)currentTime-12000) / 3000.0f, 4) + 6 * pow(((float)currentTime-12000) / 3000.0f, 5)) * (45); // Example curve logic
    } else if (currentTime >= 15000) {
         targetPosition = 0.0f; // Return to initial position
    }

    // Update position PID controller
    float power = TrajectoryControllerPosition->update(targetPosition, currentPosition, dt);

    // Serial.print(">Target Position: ");
    // Serial.println(targetPosition);
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

