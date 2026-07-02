#ifndef P4P_SRC_CASCADE_POS_VEL_CONTROLLER_H_
#define P4P_SRC_CASCADE_POS_VEL_CONTROLLER_H_

#include "Arduino.h"

class CascadePosVelController {
public:
    CascadePosVelController();
    ~CascadePosVelController();
    float update(float targetPosition, float currentPosition, float dt);
    float getOutput();

private:
    float kp_pos, ki_pos, kd_pos;
    float kp_vel, ki_vel, kd_vel;
    float windupLimit_pos, saturationHigh_pos, saturationLow_pos;
    float windupLimit_vel, saturationHigh_vel, saturationLow_vel;
    float output;
};

#endif  // P4P_SRC_CASCADE_POS_VEL_CONTROLLER_H_