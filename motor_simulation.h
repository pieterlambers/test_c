#pragma once

// Motor simulation API

#define MOTOR_MAX_SPEED_V_PER_S 5.0    // Max speed in volts per second at full PWM
#define MOTOR_POT_MIN   1.0            // Potentiometer min voltage (for position 0)
#define MOTOR_POT_MAX   4.0            // Potentiometer max voltage (for position 100)
#define MOTOR_POS_MIN   0.0            // Min position
#define MOTOR_POS_MAX   100.0          // Max position
#define MOTOR_MEAS_NOISE_V 0.05         // Noise amplitude (V) on measured voltage

void motor_simulation_step(int pwm, double dt_s); // dt_s: step in seconds
double motor_get_pot_voltage();
void motor_simulation_reset();