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

void setLED(int pin) {
    // Turn off all LEDs first
    digitalWrite(IDLE_LED_PIN, LOW);
    digitalWrite(OPERATING_LED_PIN, LOW);
    digitalWrite(ERROR_LED_PIN, LOW);
    // digitalWrite(HOMING_LED_PIN, LOW); (don't think we ever turn it off)

    // Turn on the specified LED
    digitalWrite(pin, HIGH);
}
