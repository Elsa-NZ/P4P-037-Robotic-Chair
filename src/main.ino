#include "Arduino.h"
#include <Adafruit_AS5600.h>
#include "definitions.h"
#include "MotorEncoder.h"
#include "CascadePosVelController.h"
#include "ShaftEncoder.h"
#include "BackdrivableController.h"

CascadePosVelController controller;   
BackdrivableController backdrivableController;

Adafruit_AS5600 as5600;
int homeADCValue = 0;
volatile int32_t pulses = 0;
volatile uint8_t lastState = 0;

void setupShaftEncoder() {
  as5600.begin();
  as5600.enableWatchdog(false);
  as5600.setPowerMode(AS5600_POWER_MODE_NOM);
  as5600.setHysteresis(AS5600_HYSTERESIS_OFF);
  as5600.setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_FULL);
  as5600.setSlowFilter(AS5600_SLOW_FILTER_16X);
  as5600.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);
}

float readShaftEncoderADC() {
  if (!as5600.isMagnetDetected()) {
    return NAN;
  }
  return as5600.getRawAngle();
}

void setHomePosition() {
  homeADCValue = readShaftEncoderADC();
}

void motorEncoderSetup() {
  DDRD &= ~((1 << PD2) | (1 << PD3));
  PORTD |= (1 << PORTD2) | (1 << PORTD3);

  EICRA = (1 << ISC10) | (1 << ISC00);
  EIMSK = (1 << INT1) | (1 << INT0);

  lastState = (PIND >> 2) & 0x03;
}

void evaluateEncoder() {
  uint8_t currentState = (PIND >> 2) & 0x03;

  static const int8_t lookupTable[16] = {
    0,  1, -1,  0,
   -1,  0,  0,  1,
    1,  0,  0, -1,
    0, -1,  1,  0
  };

  uint8_t index = (lastState << 2) | currentState;
  pulses += lookupTable[index];

  lastState = currentState;
}

ISR(INT0_vect) {
  evaluateEncoder();
}

ISR(INT1_vect) {
  evaluateEncoder();
}

void setup() {
  Serial.begin(115200);

  setupShaftEncoder();
  motorEncoderSetup();

  pinMode(ENABLE_PIN, OUTPUT); // ENABLE
  pinMode(DIRECTION_PIN, OUTPUT); // DIRECTION
  pinMode(PWM_PIN, OUTPUT); // PWM
  pinMode(SPEED_PIN, INPUT); // SPEED

  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(DIRECTION_PIN, HIGH); // HIGH - CCW, LOW - CW
  
  // Set the home position for the shaft encoder
  setHomePosition();
}

void loop() {
  // float targetPosition = 0.0f;
  float targetError = 0; // 0 = no torque, full backdrivable
  float dt = 0.01f;

  // Serial.print("Shaft Position: ");
  // Serial.println(readShaftEncoderADC());

  int magneticEncoderValue = readShaftEncoderADC(); // Read the current position from the shaft encoder
    if (magneticEncoderValue == NAN) {
        // Handle the case where the magnet is not detected
        Serial.println("Magnet not detected! Cannot read position.");
        return; // Exit the loop early if there's an error
    }
    


  // run the controller update every 10 ms (defined by dt)
  if (millis() % 10 == 0) {
    // HENRY"S CASCADE CONTROLLER:
    // float power = controller.update(targetPosition, currentPosition, dt);

    int32_t pulseCopy;

    noInterrupts();
    pulseCopy = pulses;
    interrupts();

    float currentPosition = (pulseCopy / (2000.0f * 51.0f)) * 360.0f;

    // float currentPosition = (pulses/(2000.0*51.0))*360.0; // Current position from encoder
    // magneticEncoderValue -= homeADCValue; // Adjust for home position
    // float magneticEncoderAngle = magneticEncoderValue * MAGNETIC_ENCODER_RESOLUTION; // Convert to float for PID calculation

    // Serial.print("Current Position: ");
    Serial.println(currentPosition);
    // Serial.print("Magnetic Encoder Angle: ");
    // Serial.println(magneticEncoderAngle);

    // float power = backdrivableController.update(targetError, currentPosition, dt, magneticEncoderAngle);
    // // Serial.println("Power: " + String(power));

    // float error = currentPosition - magneticEncoderAngle; // Calculate the angle error for PID
    // // Serial.println("Error: " + String(error));
    
    // // error handling
    // if (isnan(power)) {
    //   Serial.println("Error: Magnet not detected. Cannot compute power.");
    //   return; // Exit the loop early if there's an error
    // }

    // // Serial.println(currentPosition);

    //  if (power < 0) {
    //   digitalWrite(DIRECTION_PIN, HIGH); // CW
    //   power = abs(power);
    // } else {
    //   digitalWrite(DIRECTION_PIN, LOW); // CCW
    // }

    // Use the output to control the motor (set PWM value)
    // analogWrite(PWM_PIN, power);
  }
}