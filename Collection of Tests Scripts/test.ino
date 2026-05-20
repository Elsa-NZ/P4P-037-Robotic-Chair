#include <Arduino.h>

#define ENABLE_PIN 2
#define PWM_PIN 3
#define DIR_PIN 4
#define CURRENT_AVG A0
#define SPEED_AVG A1

float prev_time = 0; // Variable to keep track of time for motor control
int toggle = 0; // Variable to toggle motor direction (cw/ccw)
int stop = 0; // Variable to stop the motor

void setup(){

    Serial.begin(9600);
    pinMode(PWM_PIN, OUTPUT); // Will be for PWM to control speed of motor
    pinMode(ENABLE_PIN, OUTPUT); // Will be for ENABLE 

    digitalWrite(ENABLE_PIN, HIGH); // Enable the motor

}

void loop(){

    // run at max speed entire time
    analogWrite(PWM_PIN, 40); // Set initial PWM value (0-255) (230 = 90% which should be max)
    digitalWrite(DIR_PIN, HIGH); // Set direction of motor

    // read inputs
    float current = analogRead(CURRENT_AVG); // Read current sensor value
    float speed = analogRead(SPEED_AVG); // Read speed sensor value

    

    // Convert the raw analog readings to actual current and speed values if necessary
    // Current: 0V = -5A, 4V = 5A
    float currentAmps = (current / 1023.0) * 4.0 * (5.0 / 4.0) - 5.0; // Convert to amps

    // Speed: 0V = -3000 RPM, 4V = 3000 RPM
    float speedRPM = (speed / 1023.0) * 3000.0 - 3000.0; // Convert to RPM

    // Print the current and speed values to the Serial Monitor
    Serial.print("Current (A): ");
    Serial.print(currentAmps, 2); // Print current with 2 decimal places
    Serial.print(" | Speed (RPM): ");
    Serial.println(speedRPM, 2); // Print speed with 2 decimal places


    delay(500); // Delay for 500 milliseconds (0.5 seconds)
}