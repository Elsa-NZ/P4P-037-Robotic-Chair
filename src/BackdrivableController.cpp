#include "BackdrivableController.h"
#include "PID.h"
#include "definitions.h"
// #include "ShaftEncoder.h"
// #include "MotorEncoder.h"



PID* PIDController = nullptr;

BackdrivableController::BackdrivableController() {
    // Initialize position PID gains
    kp_pos = 52.0f; // Backdrive Demo: 30.0
    ki_pos = 87.0f; // Backdrive Demo: 0.0
    kd_pos = 1.0f; //Backdrive Demo: 1.0 (keep it at 1 to prevent wobbles)
    // Initialize position PID windup and saturation limits
    windupLimit_pos = 2.0f; 
    saturationHigh_pos =  0.9*255; 
    saturationLow_pos = 0.0*255; 

    filterdOutput = 0.0f;
    prevalue1 = 0.0f;
    prevalue2 = 0.0f;
    prevalue3 = 0.0f;
    prevalue4 = 0.0f;
    prevalue5 = 0.0f;

    // Create PID controllers for position and velocity
    PIDController = new PID(kp_pos, ki_pos, kd_pos, windupLimit_pos, saturationHigh_pos, saturationLow_pos, 0);
}

BackdrivableController::~BackdrivableController() {
    delete PIDController;
}

float BackdrivableController::update(float targetTorque, float motorAngle, float dt, float shaftAngle) {
    // Update position PID controller
    float currenttorque, torqueFromChair, requiredDisplacement, torque = 0.0f;
    

    displacement = motorAngle - shaftAngle; // Calculate the angle displacement for PID
    // displacement = MA_filter(displacement); // Apply moving average filter to the displacement

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
        requiredDisplacement = 0.0096 * pow(torque, 2) + 0.6876 * torque; //requiredDisplacement = 0.0096 * pow(torque, 2) + 0.6876 * torque;
    } else {
        // requiredDisplacement = 0.0096 * pow(torque, 2) + 0.6876 * torque; // Calculate the required displacement to achieve the target position
        // requiredDisplacement = -0.0096 * pow(torque,2) + 0.3876 * torque; //requiredDisplacement = -0.0096 * pow(torque,2) + 0.6876 * torque;
        requiredDisplacement = -0.0027 * pow(torque,2) + 0.3236 * torque; //requiredDisplacement = -0.0096 * pow(torque,2) + 0.6876 * torque;
    }

    // CODE FOR DATA
    if (displacement < 0) {
        currenttorque = (-0.6876 + sqrt(0.6876 * 0.6876 + 4.0 * 0.0096 * displacement))/ (2.0 * 0.0096); // Torque measured from spring
    } else {
        currenttorque = (-0.6876 + sqrt(0.6876 * 0.6876 - 4.0 * 0.0096 * displacement))/ (2.0 * -0.0096); // Torque measured from spring
    }


    // Serial.print(millis());
    // Serial.print(", ");
    // Serial.print(shaftAngle);
    // Serial.print(", ");
    // Serial.print(currenttorque); // Torque measured from spring
    // Serial.print(", ");
    // Serial.println(torque);
    
    // requiredDisplacement += 0.5; // Backlash in spring
    // Serial.print(">Required Torque: ");
    // Serial.println(torque);
    // Serial.print(">Current Torque: ");
    // Serial.println(currenttorque);
    // Serial.print(">Required Displacement: ");
    // Serial.println(requiredDisplacement);
    // Serial.print(">Displacement: ");
    // Serial.println(displacement);
    // Serial.print(">Motor Angle: ");
    // Serial.println(motorAngle);
    // Serial.print(">Shaft Angle: ");
    // Serial.println(shaftAngle);

    // if (abs(displacement) < 0.5 && targetError == 0) { // degrees
    //     displacement = 0; 
    // }

    

    // Serial.print(">Position Output: ");
    // Serial.println(positionOutput);

    // float feedforward = targetError/DEGREES_PER_AMP; // Feedforward term to assist in reaching target position (value from testing)

    // if (positionOutput != 0) {
    //     positionOutput += 25.5*(abs(positionOutput)/positionOutput);
    // }

    // Serial.println(positionOutput);

    // Store the final output for retrieval
    // output = positionOutput - feedforward;
    // if (abs(displacement-requiredDisplacement) > 0.1) {
        float positionOutput = PIDController->update(requiredDisplacement, displacement, dt); // Note: +ve  turn CCW, -ve  turn CW (NEED TO CHECK PRIOR TESTING)
        output = positionOutput;
    // }

    // output = MA_filter(output); // Apply moving average filter to the output

    // print every 500ms
    if (millis() % 50 < 10) {
        // Serial.print(">Required Torque: ");
        // Serial.println(torque);
        // Serial.print(">Current Torque: ");
        // Serial.println(currenttorque);
        Serial.print(">Required Displacement: ");
        Serial.println(requiredDisplacement);
        Serial.print(">Displacement: ");
        Serial.println(displacement);
        // Serial.print(">Motor Angle: ");
        // Serial.println(motorAngle);
        // Serial.print(">Shaft Angle: ");
        // Serial.println(shaftAngle);
        Serial.print(">Output: ");
        Serial.println(output);
    }

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
    return displacement;
}

float BackdrivableController::MA_filter(float value) {
    filterdOutput = (value + prevalue1 + prevalue2 + prevalue3 + prevalue4 + prevalue5) / 6.0f;
    prevalue5 = prevalue4;
    prevalue4 = prevalue3;
    prevalue3 = prevalue2;
    prevalue2 = prevalue1;
    prevalue1 = value;
    return filterdOutput;
}