#ifndef P4P_SRC_MOTOR_ENCODER_H_
#define P4P_SRC_MOTOR_ENCODER_H_

#include <Arduino.h>
#include "definitions.h"

// Extern declarations - definitions are in MotorEncoder.cpp
extern volatile int32_t pulses;
extern volatile uint8_t lastState;

void motorEncoderSetup();
void evaluateEncoder();

#endif  // P4P_SRC_MOTOR_ENCODER_H_