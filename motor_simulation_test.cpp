#include <iostream>
#include "unit_test.h"
#include "motor_simulation.h"
#include "motor_simulation_test.h"

void Run_MotorTests() {
    std::cout << "Motor Simulation Unit Tests\n";

    // Reset simulation
    motor_simulation_reset();

    // Test 1: Initial position should be at MOTOR_POS_MIN, voltage at MOTOR_POT_MIN
    UT_CheckInRange((int)motor_get_pot_voltage(), (int)MOTOR_POT_MIN, 0, "Initial voltage at min");

    // Test 2: Step with max positive PWM for 1 second
    motor_simulation_step(1000, 1.0);
    double v1 = motor_get_pot_voltage();
    UT_CheckTrue("Voltage increases after max PWM step", v1 > MOTOR_POT_MIN);

    // Test 3: Step with max negative PWM for 1 second (should go back to min)
    motor_simulation_step(-1000, 1.0);
    double v2 = motor_get_pot_voltage();
    UT_CheckInRange((int)v2, (int)MOTOR_POT_MIN, 1, "Voltage returns to min after negative PWM");

    // Test 4: Step with zero PWM (should not change)
    double v_before = motor_get_pot_voltage();
    motor_simulation_step(0, 1.0);
    double v_after = motor_get_pot_voltage();
    UT_CheckInRange((int)v_after, (int)v_before, 0, "Voltage unchanged with zero PWM");

    // Test 5: Motor moves at correct speed (positive direction)
    motor_simulation_reset();
    double dt = 0.5; // seconds
    motor_simulation_step(1000, dt);
    double v_expected = MOTOR_POT_MIN + MOTOR_MAX_SPEED_V_PER_S * dt;
    double v_actual = motor_get_pot_voltage();
    UT_CheckInRange((int)(v_actual * 1000), (int)(v_expected * 1000), 2, "Motor moves at correct speed (positive)"); // tolerance: 0.002V

    // Test 6: Motor saturates at max voltage
    motor_simulation_reset();
    // Step long enough to exceed max voltage
    motor_simulation_step(1000, 10.0);
    double v_max = motor_get_pot_voltage();
    UT_CheckInRange((int)(v_max * 1000), (int)(MOTOR_POT_MAX * 1000), 1, "Motor saturates at max voltage");

    // Test 7: Motor saturates at min voltage
    // First, move to max
    motor_simulation_reset();
    motor_simulation_step(1000, 10.0);
    // Now, step negative long enough to go below min
    motor_simulation_step(-1000, 20.0);
    double v_min = motor_get_pot_voltage();
    UT_CheckInRange((int)(v_min * 1000), (int)(MOTOR_POT_MIN * 1000), 1, "Motor saturates at min voltage");

    std::cout << "Motor simulation tests complete.\n";
}