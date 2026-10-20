#ifndef POSITION_CONTROLLER_H_
#define POSITION_CONTROLLER_H_

#include "Arduino.h"

class PositionController {
public:
    PositionController();
    ~PositionController();
    float updatePositionController(float targetPosition, float currentPosition, float dt, float currentFrequency, float currentPhase);
    float getOutput();

private:
    float kp_pos, ki_pos, kd_pos;
    float kp_vel, ki_vel, kd_vel;
    float windupLimit_pos, saturationHigh_pos, saturationLow_pos;
    float windupLimit_vel, saturationHigh_vel, saturationLow_vel;
    float output;
};

#endif  // P4P_SRC_CASCADE_POS_VEL_CONTROLLER_H_