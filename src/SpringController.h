#ifndef P4P_SRC_SPRINGCONTROLLER_H_
#define P4P_SRC_SPRINGCONTROLLER_H_

#include "Arduino.h"

class SpringController {
public:
    SpringController();
    ~SpringController();
    float updateLinear(float targetPosition, float currentPosition, float dt, float magneticEncoderPosition);
    float updateNonlinear(float targetPosition, float currentPosition, float dt, float magneticEncoderPosition);
    float getOutput();
    float getError();

private:
    float kp_linear, kd_linear;
    float saturationHigh_linear, saturationLow_linear;
    float kp_nonlinear, kd_nonlinear;
    float saturationHigh_nonlinear, saturationLow_nonlinear;
    float output;
    float error;
};

#endif  // P4P_SRC_SPRINGCONTROLLER_H_