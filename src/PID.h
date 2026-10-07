#ifndef P4P_SRC_PID_H_
#define P4P_SRC_PID_H_

#include "Arduino.h"

class PID {
public:
    PID(float kp, float ki, float kd, float windupLimit, float saturationHigh, float saturationLow, bool resetIntegralOnErrorSignChange = true);
    float update(float target, float current, float dt);
    float getOutput();
    void setSaturationLimits(float saturationHigh, float saturationLow);
    void setGains(float kp, float ki, float kd, float windupLimit);

private:
    float kp, ki, kd;
    float windupLimit;
    float saturationHigh, saturationLow;
    float integral, derivative;
    float lastError;
    float output;
    bool resetIntegralOnErrorSignChange;
};

#endif  // P4P_SRC_PID_H_