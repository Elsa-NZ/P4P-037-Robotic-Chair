#include "HardwareFunctions.h"

void setupHardwareState() {
    pinMode(ENABLE_SWITCH_PIN, INPUT);
    pinMode(DIP_SWITCH_PIN_0, INPUT);
    pinMode(DIP_SWITCH_PIN_1, INPUT);
    pinMode(DIP_SWITCH_PIN_2, INPUT);
    pinMode(IDLE_LED_PIN, OUTPUT);
    pinMode(OPERATING_LED_PIN, OUTPUT);
    pinMode(ERROR_LED_PIN, OUTPUT);
    pinMode(HOMING_LED_PIN, OUTPUT);
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

void LEDOn(int pin) {
    // Turn on the specified LED
    digitalWrite(pin, HIGH);
}

void LEDOff(int pin) {
    // Turn off the specified LED
    digitalWrite(pin, LOW);
}
