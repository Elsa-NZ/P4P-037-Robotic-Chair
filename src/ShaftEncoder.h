#ifndef P4P_SRC_SHAFT_ENCODER_H_
#define P4P_SRC_SHAFT_ENCODER_H_

#include <Arduino.h>
#include <Adafruit_AS5600.h>

Adafruit_AS5600 as5600;

void setupShaftEncoder() {
  as5600.begin();

  as5600.enableWatchdog(false);
  // Normal (high) power mode
  as5600.setPowerMode(AS5600_POWER_MODE_NOM);
  // No Hysteresis
  as5600.setHysteresis(AS5600_HYSTERESIS_OFF);
  // analog output
  as5600.setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_FULL);
  // setup filters
  as5600.setSlowFilter(AS5600_SLOW_FILTER_16X);
  as5600.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);
}

float readShaftEncoderADC() {
  if (!as5600.isMagnetDetected()) {
    return NAN; // Return NaN if no magnet is detected
  }
  return as5600.getRawAngle();
}

#endif  // P4P_SRC_SHAFT_ENCODER_H_
