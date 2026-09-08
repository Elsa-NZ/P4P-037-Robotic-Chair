#ifndef P4P_SRC_DEFINITIONS_H_
#define P4P_SRC_DEFINITIONS_H_

#define ENCODER_A_PIN 3
#define ENCODER_B_PIN 2
#define ENABLE_PIN A0
#define DIRECTION_PIN 4
#define PWM_PIN 5
#define SPEED_PIN A1
// #define ON_OFF_BUTTON 8

#define SHAFT_HOME_POSITION 1562.0f //2048.0f, 1531.0f, 1486.0f, 1490.0f
#define HOME_POSITION_TOLERANCE 10.0f
#define MAGNETIC_ENCODER_RESOLUTION 0.087890625f

#define ENABLE_SWITCH_PIN 6
#define DIP_SWITCH_PIN_0 9
#define DIP_SWITCH_PIN_1 8
#define DIP_SWITCH_PIN_2 7
#define IDLE_LED_PIN 12
#define OPERATING_LED_PIN 11
#define ERROR_LED_PIN 13
#define HOMING_LED_PIN 10

#define SAFE_MIN -45.0f // Minimum safe position in degrees
#define SAFE_MAX 25.0f  // Maximum safe position in degrees

#define DEGREES_PER_AMP 8.19647

#define CHAIRBACK_WEIGHT_IN_NEWTON 32.40243
#define BAR_CENTROID 0.17 // in meters (0.28) (0.34) (OG: 0.2)
#define SPRING_CONSTANT 1.05 // in Nm/deg

#define COMPRESSION_RATIO 5.0f // Define the compression ratio for the spring

#define MOTOR_POSITION_FILTER_BUFFER_SIZE 1 // Define the size of the moving average buffer for motor position

// Definitions for feedforward control parameters
#define amplitude 35.0f // Amplitude of the sine wave for feedforward control
#define k (0.5f - 0.0f) / 100.0f // Rate of change of frequency for the chirp sweep signal

// Definitions for speed cals
#define V_min 0.0f // Minimum voltage for speed calculation
#define V_max 4.0f // Maximum voltage for speed calculation
#define RPM_min -750.0f // Minimum RPM for speed calculation
#define RPM_max 750.0f // Maximum RPM for speed calculation



#endif // P4P_SRC_DEFINITIONS_H_