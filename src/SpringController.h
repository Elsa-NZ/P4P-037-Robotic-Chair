#ifndef P4P_SRC_SPRINGCONTROLLER_H_
#define P4P_SRC_SPRINGCONTROLLER_H_

#include "Arduino.h"

class SpringController {
public:
    SpringController();
    ~SpringController();
    float updateLinear(float springConstant, float equilibriumPosition, float motorPosition, float dt, float shaftPosition);
    float updateNonlinear(float targetPosition, float currentPosition, float dt, float magneticEncoderPosition);
    float SCurveTraj(float springConstant, float startPos, float endPos, float travelTime, float dt, unsigned long startTime, float motorPosition, float shaftPosition);
    float getOutput();
    float getError();

private:
    float kp_linear, ki_linear, kd_linear, kf_linear;
    float saturationHigh_linear, saturationLow_linear;
    float kp_nonlinear, ki_nonlinear, kd_nonlinear;
    float saturationHigh_nonlinear, saturationLow_nonlinear;
    float output;
    float error;
};

#endif  // P4P_SRC_SPRINGCONTROLLER_H_