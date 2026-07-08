#ifndef HARDWARE_FUNCTIONS_H
#define HARDWARE_FUNCTIONS_H
#include "definitions.h"
#include <Arduino.h>

void setupHardwareState();
bool readEnableButton();
int readSwitchDIP(); // 3 bit Switch
void setLED(int pin);

#endif  // HARDWARE_FUNCTIONS_H