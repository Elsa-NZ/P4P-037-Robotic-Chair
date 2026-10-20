#include "Motor.h"

// Global variable definitions
bool homed = false;
bool homingStarted = false;
int zeroShaftPosition = 0; // Store calibrated zero/home shaft position

void motorSetup() {
  pinMode(ENABLE_PIN, OUTPUT); // ENABLE
  digitalWrite(ENABLE_PIN, HIGH); // Enable motor driver

  pinMode(DIRECTION_PIN, OUTPUT); // DIRECTION
  pinMode(PWM_PIN, OUTPUT); // PWM
  pinMode(SPEED_PIN, INPUT); // SPEED

  if (readShaftEncoderADC() < SHAFT_HOME_POSITION) {
    digitalWrite(DIRECTION_PIN, LOW); // HIGH - CCW, LOW - CW
  } else {
    digitalWrite(DIRECTION_PIN, HIGH); // HIGH - CCW, LOW - CW
  }
}

void move(float power) {

    

    if (isnan(power)) { // If power is NaN or below threshold, set to minimum PWM value
        power = 0.1*255; // Set a minimum PWM value to prevent controller error
    } else if (power > 0) { // If power is below threshold, set to minimum PWM value
      digitalWrite(DIRECTION_PIN, LOW); // CCW  
      power += 0.1*255;
    } else if (power < 0) {
      digitalWrite(DIRECTION_PIN, HIGH); // CW
      power = abs(power - 0.1*255);
    }

    if (power > 255*0.9) { // If power is above threshold, set to maximum PWM value
        power = 0.9*255; // Set a maximum PWM value to prevent controller error
    } else if (power < 0.1*255) { // If power is below threshold, set to minimum PWM value
        power = 0.1*255; // Set a minimum PWM value to prevent controller error
    }
    
    analogWrite(PWM_PIN, power);
}


PID homingPID(0.07, 0.066, 0.01, 140, 0.27*255, 0.0, 1); // homingPID(kp, ki, kd, integral_windup, pwm_max, pwm_min // 0.057, 0.18, 0.01, 200, 0.27*255, 0.12*255, 1
unsigned long previousMillis = 0;
int timerStart = 0;
unsigned long timer = 0;

int home() {
  if (!homingStarted) {
    homingStarted = true;
    previousMillis = millis();
  }

  int position = readShaftEncoderADC();
  

  if ((position <= SHAFT_HOME_POSITION + HOME_POSITION_TOLERANCE && position >= SHAFT_HOME_POSITION - HOME_POSITION_TOLERANCE)) { // SOMETHING IS WRONG WITH SPEED
    if (timerStart == 0) {
      timer = millis();
      timerStart = 1;
    }
    
    if (millis() - timer > 1000) {
      homed = true;
      homingStarted = false;
      analogWrite(PWM_PIN, 0.1*255);
      delay(1000); // Allow time for motor to settle
      pulses = 0; // Reset encoder pulses to zero at home position
      zeroShaftPosition = readShaftEncoderADC();; // Store the zero shaft position
      return 1; // Already at home position
    }
  } else {
    timerStart = 0; // Reset timer if not at home position
  }

  float dt = (millis() - previousMillis) / 1000.0; // Convert to seconds
  previousMillis = millis();

  float power = homingPID.update(SHAFT_HOME_POSITION, position, dt);


  move(power);

  return 0; // Not at home position
}