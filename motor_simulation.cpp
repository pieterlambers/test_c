#include "motor_simulation.h"

// Simple motor simulation

static double motor_position = 0.0; // Position in arbitrary units
static double motor_speed = 0.0;    // Speed in units/ms
static const double max_speed = 0.005; // Max speed per ms at full PWM
static const double pot_min = 0.0;  // Potentiometer min voltage
static const double pot_max = 5.0;  // Potentiometer max voltage
static const double pos_min = 0.0;  // Min position (maps to 0V)
static const double pos_max = 1000.0; // Max position (maps to 5V)

// Simulate one time step (dt in ms), pwm in range [-1000, 1000]
void motor_simulation_step(int pwm, double dt_ms) {
    // Clamp PWM
    if (pwm > 1000) pwm = 1000;
    if (pwm < -1000) pwm = -1000;

    // Calculate speed proportional to PWM
    motor_speed = (pwm / 1000.0) * max_speed;

    // Integrate position
    motor_position += motor_speed * dt_ms;

    // Clamp position to valid range
    if (motor_position < pos_min) motor_position = pos_min;
    if (motor_position > pos_max) motor_position = pos_max;
}

// Get potentiometer voltage (0-5V) based on position
double motor_get_pot_voltage() {
    double voltage = pot_min + (motor_position - pos_min) * (pot_max - pot_min) / (pos_max - pos_min);
    if (voltage < pot_min) voltage = pot_min;
    if (voltage > pot_max) voltage = pot_max;
    return voltage;
}

// Optionally, reset simulation
void motor_simulation_reset() {
    motor_position = 0.0;
    motor_speed = 0.0;
}