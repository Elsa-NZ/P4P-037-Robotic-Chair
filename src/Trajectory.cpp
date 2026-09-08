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
    ki_pos = 5.5f; 
    kd_pos = 0.1f; 

    // Gain scheduling for position PID based on velocity
    // 35 deg/s: kp = 5.5, ki = 5.5, kd = 0.1
    // 16.67 deg/s: kp = 4.0, ki = 5.5, kd = 0.1
    // 9 deg/s: kp = 2.0, ki = 5.5, kd = 0.15


    // Initialize position PID windup and saturation limits
    windupLimit_pos = 25.0f; 
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
    long currentTime = millis() - time+3500; // Current time in milliseconds since the start of the trajectory

    float ka_linear = 0.1f; // Feedforward gain acceleration
    float kv_linear = 0.5f; // Feedforward gain velocity
    float ks_linear = 1.25f; // Feedforward gain angle



    /*if (currentTime < 3000) {
        s = (float)currentTime / 3000.0f;
        targetPosition = -50.0f * (10*s*s*s - 15*s*s*s*s + 6*s*s*s*s*s);
        velocity = (-50.0f / 3000.0f) * (30*s*s - 60*s*s*s + 30*s*s*s*s);
        acceleration = (-50.0f / (3000.0f * 3000.0f)) * (60*s - 180*s*s + 120*s*s*s);
    } else if (currentTime >= 3000 && currentTime < 5000) {
        targetPosition = -50.0f; // stay in position for 2 seconds
        velocity = 0.0f; // No change in position, so velocity is zero
        acceleration = 0.0f; // No change in velocity, so acceleration is zero
    } else*/ if (currentTime >= 3500 && currentTime < 5500) {
        delta = -45.0f - 25.0f;
        s = ((float)currentTime - 3500.0f) / 2000.0f;
        targetPosition = /*25.0f*/ + delta * (10*s*s*s - 15*s*s*s*s + 6*s*s*s*s*s);
        velocity = (delta / 2000.0f) * (30*s*s - 60*s*s*s + 30*s*s*s*s);
        acceleration = (delta / (2000.0f * 2000.0f)) * (60*s - 180*s*s + 120*s*s*s);
    } else if (currentTime >= 5500 && currentTime < 7500) {
        targetPosition = -70.0f; // stay in position for 2 seconds
        velocity = 0.0f; // No change in position, so velocity is zero
        acceleration = 0.0f; // No change in velocity, so acceleration is zero
    } //else if (currentTime >= 7500 && currentTime < 12500) {
    //     float time = 5000.0f;
    //     delta = 45.0f; // final - initial = 0 - (-45)
    //     s = ((float)currentTime - 7500.0f) / time;
    //     targetPosition = -45.0f + delta * (10*s*s*s - 15*s*s*s*s + 6*s*s*s*s*s);
    //     velocity = (delta / time) * (30*s*s - 60*s*s*s + 30*s*s*s*s);
    //     acceleration = (delta / (time * time)) * (60*s - 180*s*s + 120*s*s*s);
    // } else if (currentTime >= 12500) {
    //     targetPosition = 45.0f; // Return to initial position
    //     velocity = 0.0f; // No change in position, so velocity is zero
    //     acceleration = 0.0f; // No change in velocity, so acceleration is zero
    // }

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
         Serial.print(">Power: ");
        Serial.println(position + feedforward);
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

float Trajectory::SCurveTraj(float startPos, float endPos, float travelTime, float dt, unsigned long startTime, float ShaftPosition) {
    // Implement the logic for following an S-curve trajectory
    // This is a placeholder implementation and should be replaced with actual S-curve-following logic


    float avgVelocity = (endPos - startPos) / travelTime; // Define the velocity based on the S-curve trajectory
    float delta = endPos - startPos; // Define the change in position based on the S-curve trajectory

    // feedfoward gains:
    float ka_linear = 0.1f; // Feedforward gain acceleration
    float kv_linear = 0.5f; // Feedforward gain velocity
    float ks_linear = 1.25f; // Feedforward gain angle

    // Gain scheduling for position PID based on velocity
    // kp_pos = 0.1248*avgVelocity + 1.3129;
    // ki_pos = 5.5;
    // kd_pos = -0.0016*avgVelocity + 0.1483;

    // kp_pos = -0.0015*delta*delta + 0.1307*delta + 0.6163; //0.0522*avgVelocity + 1.3917
    // ki_pos = 0.0037*delta + 1.3905;// -0.0103*avgVelocity + 1.4495
    // kd_pos = 0.0004*delta + 0.1053; //0.002*avgVelocity + 0.0775

    kp_pos = 3.7;
    ki_pos = 1.0;
    kd_pos = 0.12;

    TrajectoryControllerPosition->setGains(kp_pos, ki_pos, kd_pos);

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

    Serial.print(">Target Position: ");
    Serial.println(targetPosition);

    float position = TrajectoryControllerPosition->update(targetPosition, ShaftPosition, dt);
    double feedforward = 1000000 * ka_linear * acceleration + 1000 * kv_linear * velocity - ks_linear * CHAIRBACK_WEIGHT_IN_NEWTON * sin(targetPosition * (PI / 180.0f)) * BAR_CENTROID; // Feedforward term to assist in reaching target position (value from testing)

    float power = position + feedforward;

    Serial.print(">Power: ");
    Serial.println(power);

    return power;
}
