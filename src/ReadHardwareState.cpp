#include "ReadHardwareState.h"

void setupHardwareState() {
    pinMode(ENABLE_SWITCH_PIN, INPUT);
    pinMode(DIP_SWITCH_PIN_0, INPUT);
    pinMode(DIP_SWITCH_PIN_1, INPUT);
    pinMode(DIP_SWITCH_PIN_2, INPUT);
}

bool readEnableButton(){
    return digitalRead(ENABLE_SWITCH_PIN) == HIGH;
}

int readSwitchDIP(){ // 3 bit Switch
    bool bit0 = digitalRead(DIP_SWITCH_PIN_0) == HIGH;
    bool bit1 = digitalRead(DIP_SWITCH_PIN_1) == HIGH;
    bool bit2 = digitalRead(DIP_SWITCH_PIN_2) == HIGH;

    return (bit2 << 2) | (bit1 << 1) | bit0;
}
