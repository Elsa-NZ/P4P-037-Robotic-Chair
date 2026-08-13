#ifndef P4P_SRC_DEFINITIONS_H_
#define P4P_SRC_DEFINITIONS_H_

#define ENCODER_A_PIN 3
#define ENCODER_B_PIN 2
#define ENABLE_PIN A0
#define DIRECTION_PIN 4
#define PWM_PIN 5
#define SPEED_PIN A1
// #define ON_OFF_BUTTON 8

#define SHAFT_HOME_POSITION 1490.0f //2048.0f, 1531.0f, 1486.0f
#define HOME_POSITION_TOLERANCE 5.0f
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
#define BAR_CENTROID 0.2 // in meters (0.28) (0.34)
#define SPRING_CONSTANT 1.05 // in Nm/deg

#define COMPRESSION_RATIO 5.0f // Define the compression ratio for the spring

#define MOTOR_POSITION_FILTER_BUFFER_SIZE 1 // Define the size of the moving average buffer for motor position

#endif // P4P_SRC_DEFINITIONS_H_