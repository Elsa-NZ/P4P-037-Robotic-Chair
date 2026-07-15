#include "PID.h"

PID::PID(float kp, float ki, float kd, float windupLimit, float saturationHigh, float saturationLow, bool resetIntegralOnErrorSignChange) {
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
    this->resetIntegralOnErrorSignChange = resetIntegralOnErrorSignChange;
}

float PID::update(float target, float current, float dt) {
    float error = target - current;

    Serial.println(error);

    // Proportional term
    float proportionalTerm = kp * error;

    // Reset the integral when the error crosses zero, including through an exact zero sample.
    bool errorCrossedZero = (error > 0.0f && lastError <= 0.0f) || (error < 0.0f && lastError >= 0.0f);
    if (resetIntegralOnErrorSignChange && errorCrossedZero) {
        integral = 0.0f;
        // Serial.println("Integral reset due to error sign change");
    }

    // Integral term with windup protection
    // Serial.println("Integral before update: " + String(integral));
    integral += error * dt;
    // Serial.println("Integral after update: " + String(integral));
    // Serial.println("dt: " + String(dt));
    if (integral > windupLimit) {
        integral = windupLimit;
    } else if (integral < -windupLimit) {
        integral = -windupLimit;
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
    if (output > saturationHigh) {
        output = saturationHigh;
    } else if (output < saturationLow && output > 0) {
        output = saturationLow;
    } else if (output > -saturationLow && output < 0) {
        output = -saturationLow;
    } else if (output < -saturationHigh) {
        output = -saturationHigh;
    }

    lastError = error;
    return output;
}

float PID::getOutput() {
    return output;
}

void PID::setSaturationLimits(float saturationHigh, float saturationLow) {
    this->saturationHigh = saturationHigh;
    this->saturationLow = saturationLow;
}