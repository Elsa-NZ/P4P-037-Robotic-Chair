#ifndef P4P_SRC_BACKDRIVABLECONTROLLER_H_
#define P4P_SRC_BACKDRIVABLECONTROLLER_H_

#include "Arduino.h"

class BackdrivableController {
public:
    BackdrivableController();
    ~BackdrivableController();
    float update(float targetPosition, float currentPosition, float dt);
    float getOutput();

private:
    float kp_pos, ki_pos, kd_pos;
    float windupLimit_pos, saturationHigh_pos, saturationLow_pos;
    float output;
};

#endif  // P4P_SRC_BACKDRIVABLECONTROLLER_H_