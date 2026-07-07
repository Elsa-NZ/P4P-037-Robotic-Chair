#ifndef READ_HARDWARE_STATE_H
#define READ_HARDWARE_STATE_H
#include "definitions.h"
#include <Arduino.h>

void setupHardwareState();
bool readEnableButton();
int readSwitchDIP(); // 3 bit Switch

#endif  // READ_HARDWARE_STATE_H