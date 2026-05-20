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

    if (millis() - prev_time >= 500 && millis() - prev_time < 1000) {
        
        analogWrite(PWM_PIN, 5); // Set initial PWM value (0-255) (230 = 90% which should be max)
        stop = 1; // Stop the motor
        
    } else if (millis() - prev_time >= 1000) {
        toggle = !toggle; // Toggle the state
        prev_time = millis(); // Reset the timer
        stop = 0;
        }

    if (toggle && !stop) { // Run the motor for 5 seconds
        analogWrite(PWM_PIN, 230); // Set initial PWM value (0-255) (230 = 90% which should be max)
        digitalWrite(DIR_PIN, HIGH); // Set direction of motor
    } else if (!toggle && !stop) { // Run the motor in the opposite direction 
        analogWrite(PWM_PIN, 230); // Stop the motor
        digitalWrite(DIR_PIN, LOW); // Set direction of motor
    }

    int current = analogRead(CURRENT_AVG); // Read current sensor value
    int speed = analogRead(SPEED_AVG); // Read speed sensor value

    // Convert the raw analog readings to actual current and speed values if necessary
    // Current: 0V = 0A, 4V = 3A
    float currentAmps = (current / 1023.0) * 4.0 * (3.0 / 4.0); // Convert to amps

    // Speed: 0V = 0 RPM, 4V = 60 RPM
    float speedRPM = (speed / 1023.0) * 60.0; // Convert to RPM

    // Print the current and speed values to the Serial Monitor
    Serial.print("Current (A): ");
    Serial.print(currentAmps, 2); // Print current with 2 decimal places
    Serial.print(" | Speed (RPM): ");
    Serial.println(speedRPM, 2); // Print speed with 2 decimal places


    delay(500); // Delay for 500 milliseconds (0.5 seconds)
}