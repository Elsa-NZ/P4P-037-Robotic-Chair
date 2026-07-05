#include "BackdrivableController.h"
#include "PID.h"
#include "definitions.h"
#include "ShaftEncoder.h"
#include "MotorEncoder.h"



PID* positionPID = nullptr;

BackdrivableController::BackdrivableController() {
    // Initialize position PID gains
    kp_pos = 1.0f;
    ki_pos = 0.0f; 
    kd_pos = 0.0f; 
    // Initialize position PID windup and saturation limits
    windupLimit_pos = 60.0f; 
    saturationHigh_pos = 400.0f; 
    saturationLow_pos = -400.0f; 

    // Create PID controllers for position and velocity
    positionPID = new PID(kp_pos, ki_pos, kd_pos, windupLimit_pos, saturationHigh_pos, saturationLow_pos);
}

BackdrivableController::~BackdrivableController() {
    delete positionPID;
}

float BackdrivableController::update(float targetError, float currentPosition, float dt) {
    // Update position PID controller
    int magneticEncoderValue = readShaftEncoderADC(); // Read the current position from the shaft encoder
    if (magneticEncoderValue == NAN) {
        // Handle the case where the magnet is not detected
        Serial.println("Magnet not detected! Cannot read position.");
        return NAN; // Return NaN to indicate an error
    }
    magneticEncoderValue -= homeADCValue; // Adjust for home position
    float magneticEncoderAngle = magneticEncoderValue * MAGNETIC_ENCODER_RESOLUTION; // Convert to float for PID calculation

    float error = currentPosition - magneticEncoderAngle; // Calculate the angle error for PID

    float positionOutput = positionPID->update(targetError, error, dt); // Note: +ve  turn CCW, -ve  turn CW (NEED TO CHECK PRIOR TESTING)

    // if (positionOutput != 0) {
    //     positionOutput += 25.5*(abs(positionOutput)/positionOutput);
    // }

    Serial.println(positionOutput);

    // Store the final output for retrieval
    output = positionOutput;

    return output;
}

float BackdrivableController::getOutput() {
    return output;
}