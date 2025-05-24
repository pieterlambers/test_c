#include <cstdlib>
#include "motor_simulation.h"

// Simple motor simulation

static double motor_position = MOTOR_POS_MIN; // Position in range 0..100
static double motor_speed_v = 0.0;            // Speed in volts/second

#define VOLTS_PER_POS ((MOTOR_POT_MAX - MOTOR_POT_MIN) / (MOTOR_POS_MAX - MOTOR_POS_MIN))

// Simulate one time step (dt in seconds), pwm in range [-1000, 1000]
void motor_simulation_step(int pwm, double dt_s) {
    // Clamp PWM
    if (pwm > 1000) pwm = 1000;
    if (pwm < -1000) pwm = -1000;

    // Calculate speed proportional to PWM (in volts/second)
    motor_speed_v = (pwm / 1000.0) * MOTOR_MAX_SPEED_V_PER_S;

    // Calculate position delta directly (position units per second)
    double motor_speed_pos = motor_speed_v / VOLTS_PER_POS; // position units per second

    // Integrate position
    motor_position += motor_speed_pos * dt_s;

    // Clamp position to valid range
    if (motor_position < MOTOR_POS_MIN) motor_position = MOTOR_POS_MIN;
    if (motor_position > MOTOR_POS_MAX) motor_position = MOTOR_POS_MAX;
}

// Get potentiometer voltage based on position, with optional simple noise
double motor_get_pot_voltage() {
    double voltage = MOTOR_POT_MIN + (motor_position - MOTOR_POS_MIN) * VOLTS_PER_POS;

    // Simple uniform noise in range [-MOTOR_MEAS_NOISE_V, +MOTOR_MEAS_NOISE_V]
    voltage += ((2.0 * rand() / RAND_MAX) - 1.0) * MOTOR_MEAS_NOISE_V;
    return voltage;
}

// Optionally, reset simulation
void motor_simulation_reset() {
    motor_position = MOTOR_POS_MIN;
    motor_speed_v = 0.0;
}