#include "motor_simulation.h"

// Simple motor simulation

static double motor_position = 0.0; // Position in arbitrary units
static double motor_speed = 0.0;    // Speed in units/ms

// Use #defines from header directly, do not keep local const variables

// Simulate one time step (dt in ms), pwm in range [-1000, 1000]
void motor_simulation_step(int pwm, double dt_ms) {
    // Clamp PWM
    if (pwm > 1000) pwm = 1000;
    if (pwm < -1000) pwm = -1000;

    // Calculate speed proportional to PWM
    motor_speed = (pwm / 1000.0) * MOTOR_MAX_SPEED;

    // Integrate position
    motor_position += motor_speed * dt_ms;

    // Clamp position to valid range
    if (motor_position < MOTOR_POS_MIN) motor_position = MOTOR_POS_MIN;
    if (motor_position > MOTOR_POS_MAX) motor_position = MOTOR_POS_MAX;
}

// Get potentiometer voltage (0-5V) based on position
double motor_get_pot_voltage() {
    double voltage = MOTOR_POT_MIN + (motor_position - MOTOR_POS_MIN) * (MOTOR_POT_MAX - MOTOR_POT_MIN) / (MOTOR_POS_MAX - MOTOR_POS_MIN);
    if (voltage < MOTOR_POT_MIN) voltage = MOTOR_POT_MIN;
    if (voltage > MOTOR_POT_MAX) voltage = MOTOR_POT_MAX;
    return voltage;
}

// Optionally, reset simulation
void motor_simulation_reset() {
    motor_position = 0.0;
    motor_speed = 0.0;
}