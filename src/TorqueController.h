#ifndef TORQUE_CONTROLLER_H_
#define TORQUE_CONTROLLER_H_

#include "Arduino.h"

class TorqueController {
public:
    TorqueController();
    ~TorqueController();
    float Spring(float springConstant, float equilibriumPosition, float motorPosition, float dt, float shaftPosition);
    float SCurveTraj(float startPos, float endPos, float travelTime, float dt, unsigned long startTime, float motorPosition, float shaftPosition);
    float updateTorqueController(float motorPosition, float dt, float shaftPosition, float targetTorque);

private:
    float kp_linear, ki_linear, kd_linear, kf_linear;
    float saturationHigh_linear, saturationLow_linear;
    float output;
};

#endif 