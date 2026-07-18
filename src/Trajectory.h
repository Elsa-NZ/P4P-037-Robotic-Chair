#ifndef P4P_SRC_TRAJECTORY_H_
#define P4P_SRC_TRAJECTORY_H_

#include "Arduino.h"

static unsigned long lastTime = 0; // Variable to store the last time the function was called

class Trajectory {
public:
    Trajectory();
    ~Trajectory();
    float followRampTrajectory(float shaftPosition, float dt, unsigned long time);
    float followCurveTrajectory(float shaftPosition, float dt, unsigned long time);

private:
    float kp_pos, ki_pos, kd_pos, kp_vel, ki_vel, kd_vel;
    float windupLimit_vel, saturationHigh_vel, saturationLow_vel;
    float windupLimit_pos, saturationHigh_pos, saturationLow_pos;
    float output;
    float error;
};

#endif  // P4P_SRC_TRAJECTORY_H_