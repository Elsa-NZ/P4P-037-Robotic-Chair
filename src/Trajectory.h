#ifndef P4P_SRC_TRAJECTORY_H_
#define P4P_SRC_TRAJECTORY_H_

#include "Arduino.h"


class Trajectory {
public:
    Trajectory();
    ~Trajectory();
    float PosSCurveTraj(float startPos, float endPos, float travelTime, float dt, unsigned long startTime, float ShaftPosition);

private:
    float kp_pos, ki_pos, kd_pos;
    float windupLimit_pos, saturationHigh_pos, saturationLow_pos;
    float output;
    float error;
};

#endif  // P4P_SRC_TRAJECTORY_H_