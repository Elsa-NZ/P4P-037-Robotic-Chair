#ifndef P4P_SRC_SHAFT_ENCODER_H_
#define P4P_SRC_SHAFT_ENCODER_H_

#include <Arduino.h>
#include <Adafruit_AS5600.h>

extern Adafruit_AS5600 as5600;

void shaftEncoderSetup();
float readShaftEncoderADC();

#endif  // P4P_SRC_SHAFT_ENCODER_H_
