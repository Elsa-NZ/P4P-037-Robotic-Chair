#include "Arduino.h"
#include <Adafruit_AS5600.h>
#include "definitions.h"
#include "MotorEncoder.h"
#include "CascadePosVelController.h"
#include "ShaftEncoder.h"
#include "BackdrivableController.h"

CascadePosVelController controller;   
BackdrivableController backdrivableController;

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
}

void loop() {
  // float targetPosition = 0.0f;
  float targetError = 0; // 0 = no torque, full backdrivable
  float dt = 0.01f;

  Serial.print("Shaft Position: ");
  Serial.println(readShaftEncoderADC());

  // run the controller update every 10 ms (defined by dt)
  if (millis() % 10 == 0) {
    // HENRY"S CASCADE CONTROLLER:
    // float power = controller.update(targetPosition, currentPosition, dt);

    float currentPosition = (pulses/(2000.0*51.0))*360.0; // Current position from encoder
    float power = backdrivableController.update(targetError, currentPosition, dt);
    Serial.println("Power: " + String(power));
    float magneticEncoderValue = readShaftEncoderADC();
    magneticEncoderValue -= homeADCValue; // Adjust for home position
    float magneticEncoderAngle = magneticEncoderValue * MAGNETIC_ENCODER_RESOLUTION; // Convert to float for PID calculation

    float error = currentPosition - magneticEncoderAngle; // Calculate the angle error for PID
    Serial.println("Error: " + String(error));
    
    // error handling
    if (isnan(power)) {
      Serial.println("Error: Magnet not detected. Cannot compute power.");
      return; // Exit the loop early if there's an error
    }

    // Serial.println(currentPosition);

     if (power < 0) {
      digitalWrite(DIRECTION_PIN, HIGH); // CW
      power = abs(power);
    } else {
      digitalWrite(DIRECTION_PIN, LOW); // CCW
    }

    // Use the output to control the motor (set PWM value)
    // analogWrite(PWM_PIN, power);
  }
}