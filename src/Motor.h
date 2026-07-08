#ifndef P4P_SRC_MOTOR_H_
#define P4P_SRC_MOTOR_H_

#include <Arduino.h>
#include "definitions.h"
#include "PID.h"
#include "ShaftEncoder.h"
#include "MotorEncoder.h"

extern bool homed;
extern bool homingStarted;

void motorSetup();
void move(float power);
int home();
#endif  // P4P_SRC_MOTOR_H_