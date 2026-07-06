#ifndef P4P_SRC_MOTOR_H_
#define P4P_SRC_MOTOR_H_

#include <Arduino.h>
#include "definitions.h"
#include "PID.h"
#include "ShaftEncoder.h"

bool homed = false;
bool homingStarted = false;

void motorSetup() {
  pinMode(ENABLE_PIN, OUTPUT); // ENABLE
  pinMode(DIRECTION_PIN, OUTPUT); // DIRECTION
  pinMode(PWM_PIN, OUTPUT); // PWM
  pinMode(SPEED_PIN, INPUT); // SPEED

  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(DIRECTION_PIN, HIGH); // HIGH - CCW, LOW - CW
}

PID homingPID(0.9, 0.1, 0.0, 10, 0.2*255, 0.1*255); // homingPID(kp, ki, kd, integral_windup, pwm_max, pwm_min
int previousMillis = 0;

int home() {
  if (!homingStarted) {
    homingStarted = true;
    previousMillis = millis();
  }

  int position = readShaftEncoderADC();

  if (position <= SHAFT_HOME_POSITION + HOME_POSITION_TOLERANCE && position >= SHAFT_HOME_POSITION - HOME_POSITION_TOLERANCE) {
    homed = true;
    homingStarted = false;
    return 1; // Already at home position
  }

  float dt = (millis() - previousMillis) / 1000.0; // Convert to seconds
  previousMillis = millis();

  float power = homingPID.update(SHAFT_HOME_POSITION, position, dt);

  move(power);

  return 0; // Not at home position
}

void move(float power) {
    if (power < 0) {
      digitalWrite(DIRECTION_PIN, HIGH); // CW
      power = abs(power);
    } else {
      digitalWrite(DIRECTION_PIN, LOW); // CCW
    }

    // Use the output to control the motor (set PWM value)
    analogWrite(PWM_PIN, power);
}

#endif  // P4P_SRC_MOTOR_H_