#ifndef P4P_SRC_BACKDRIVABLECONTROLLER_H_
#define P4P_SRC_BACKDRIVABLECONTROLLER_H_

#include "Arduino.h"

class BackdrivableController {
public:
    BackdrivableController();
    ~BackdrivableController();
    float update(float targetPosition, float currentPosition, float dt, float magneticEncoderPosition);
    int calibrate();
    float getOutput();
    float getError();

private:
    float kp_pos, ki_pos, kd_pos;
    float windupLimit_pos, saturationHigh_pos, saturationLow_pos;
    float output;
    float error;
    float MA_filter(float value);
    float filterdOutput, prevalue1, prevalue2, prevalue3, prevalue4, prevalue5;
};

#endif  // P4P_SRC_BACKDRIVABLECONTROLLER_H_