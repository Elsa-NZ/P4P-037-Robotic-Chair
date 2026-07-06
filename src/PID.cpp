#include "PID.h"

PID::PID(float kp, float ki, float kd, float windupLimit, float saturationHigh, float saturationLow) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->windupLimit = windupLimit;
    this->saturationHigh = saturationHigh;
    this->saturationLow = saturationLow;
    this->integral = 0.0f;
    this->derivative = 0.0f;
    this->lastError = 0.0f;
    this->output = 0.0f;
}

float PID::update(float target, float current, float dt) {
    float error = target - current;

    // Serial.println(error);

    // Proportional term
    float proportionalTerm = kp * error;

    // Integral term with windup protection
    integral += error * dt;
    if (integral > windupLimit) {
        integral = windupLimit;
    } else if (integral < -windupLimit) {
        integral = -windupLimit;
    }

    if (abs(error) < 0.1) {
     integral = 0;   
    }

    float integralTerm = ki * integral;

    // Derivative term
    if (dt <= 0) {
        derivative = 0; // Prevent division by zero
    } else {
        derivative = (error - lastError) / dt;
    }
    
    float derivativeTerm = kd * derivative;

    // Calculate output and apply saturation limits
    output = proportionalTerm + integralTerm + derivativeTerm;
    if (abs(output) > saturationHigh) {
        output = saturationHigh*(output/abs(output));
    } else if (abs(output) < saturationLow) {
        output = saturationLow*(output/abs(output));
    }

    // Serial.println(output);

    lastError = error;
    return output;
}

float PID::getOutput() {
    return output;
}