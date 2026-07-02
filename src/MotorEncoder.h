#ifndef P4P_SRC_MOTOR_ENCODER_H_
#define P4P_SRC_MOTOR_ENCODER_H_

#include <Arduino.h>
#include "definitions.h"

// Ensure these global variables are marked volatile
volatile int32_t pulses = 0;
volatile uint8_t lastState = 0;

void motorEncoderSetup() {
  DDRD &= ~((1 << PD2) | (1 << PD3)); // Set input
  PORTD |= (1 << PORTD2) | (1 << PORTD3); // Pull-up resistors

  EICRA = (1 << ISC10) | (1 << ISC00); // Trigger any logical change

  EIMSK = (1 << INT1) | (1 << INT0); // Enable external interrupts for both INT0 (D2) and INT1 (D3)
  
  lastState = (PIND >> 2) & 0x03; // Read initial state of the pins to seed the state machine
}

// Forward declaration so ISRs can call it
void evaluateEncoder() {
  uint8_t currentState = (PIND >> 2) & 0x03; // Read pins D2 and D3 simultaneously
  
  // An array representing the direction for every possible state transition
  // 1 = Clockwise/CCW, -1 = Opposite, 0 = Invalid/Missed steps
  static const int8_t lookupTable[16] = {
    0,  1, -1,  0, 
   -1,  0,  0,  1, 
    1,  0,  0, -1, 
    0, -1,  1,  0
  };
  
  // Combine last state and current state to get a 4-bit index (0-15)
  uint8_t index = (lastState << 2) | currentState;
  pulses += lookupTable[index];
  
  lastState = currentState; // Save state for the next interrupt
}

ISR(INT0_vect) {
  evaluateEncoder();
}

ISR(INT1_vect) {
  evaluateEncoder();
}

#endif  // P4P_SRC_MOTOR_ENCODER_H_