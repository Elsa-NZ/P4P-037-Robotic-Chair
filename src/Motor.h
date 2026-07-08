#ifndef P4P_SRC_MOTOR_H_
#define P4P_SRC_MOTOR_H_

#include <Arduino.h>
#include "definitions.h"
#include "PID.h"
#include "ShaftEncoder.h"
#include "MotorEncoder.h"

bool homed = false;
bool homingStarted = false;

void motorSetup() {
  pinMode(ENABLE_PIN, OUTPUT); // ENABLE
  pinMode(DIRECTION_PIN, OUTPUT); // DIRECTION
  pinMode(PWM_PIN, OUTPUT); // PWM
  pinMode(SPEED_PIN, INPUT); // SPEED

  digitalWrite(ENABLE_PIN, HIGH);

  if (readShaftEncoderADC() < SHAFT_HOME_POSITION) {
    digitalWrite(DIRECTION_PIN, LOW); // HIGH - CCW, LOW - CW
  } else {
    digitalWrite(DIRECTION_PIN, HIGH); // HIGH - CCW, LOW - CW
  }
}

void move(float power) {
    if (isnan(power)) {
        power = 0.1*255; // Set a minimum PWM value to prevent controller error
    } else if (power < 0) {
      digitalWrite(DIRECTION_PIN, HIGH); // CW
      power = abs(power);
    } else {
      digitalWrite(DIRECTION_PIN, LOW); // CCW
    }

    // Serial.print("Motor Power: ");
    // Serial.println(power);

    // Use the output to control the motor (set PWM value)
    analogWrite(PWM_PIN, power);
}


PID homingPID(0.7, 0.22, 0.1, 300, 0.27*255, 0.1*255); // homingPID(kp, ki, kd, integral_windup, pwm_max, pwm_min
int previousMillis = 0;

int home() {
  if (!homingStarted) {
    homingStarted = true;
    previousMillis = millis();
  }

  int position = readShaftEncoderADC();

  float error = SHAFT_HOME_POSITION - position;

  // map angular error to pwm range to saturate power
  homingPID.setSaturationLimits(6.86103198783*log(abs(error))+20.9575950004, 0.1*255); // Set saturation limits for homing PID

  float speed = 1.8310546875*analogRead(SPEED_PIN)-740; //(1.8310546875*measuredVelocity)-740)

  if ((position <= SHAFT_HOME_POSITION + HOME_POSITION_TOLERANCE && position >= SHAFT_HOME_POSITION - HOME_POSITION_TOLERANCE) && abs(speed) < 0.5) {
    homed = true;
    homingStarted = false;
    analogWrite(PWM_PIN, 0.1*255);
    pulses = 0; // Reset encoder pulses to zero at home position
    return 1; // Already at home position
  }

  float dt = (millis() - previousMillis) / 1000.0; // Convert to seconds
  previousMillis = millis();

  float power = homingPID.update(SHAFT_HOME_POSITION, position, dt);

  
//   Serial.print("Shaft Speed: ");
//   Serial.println(speed);
//   Serial.print("Home Position: ");
//   Serial.println(SHAFT_HOME_POSITION);

  move(power);

  return 0; // Not at home position
}
#endif  // P4P_SRC_MOTOR_H_