#pragma once

// Motor simulation API

#define MOTOR_MAX_SPEED 0.005      // Max speed per ms at full PWM
#define MOTOR_POT_MIN   0.0        // Potentiometer min voltage
#define MOTOR_POT_MAX   5.0        // Potentiometer max voltage
#define MOTOR_POS_MIN   0.0        // Min position (maps to 0V)
#define MOTOR_POS_MAX   1000.0     // Max position (maps to 5V)

void motor_simulation_step(int pwm, double dt_ms);
double motor_get_pot_voltage();
void motor_simulation_reset();