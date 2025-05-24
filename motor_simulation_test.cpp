#include <iostream>
#include "unit_test.h"
#include "motor_simulation.h"
#include "motor_simulation_test.h"

void Run_MotorSimulation_Tests() {
    UT_SetLogFile("motor_simulation_testresults.txt");
    std::cout << "Motor Simulation Unit Tests\n";

    UT_SetTestNumber(1);
    UT_TestInfo("Initial position should be at MOTOR_POS_MIN, voltage at MOTOR_POT_MIN");
    motor_simulation_reset();
    // Allow for noise in the initial voltage
    UT_CheckInRange((int)(motor_get_pot_voltage() * 1000), (int)(MOTOR_POT_MIN * 1000), 55, "Initial voltage at min");

    UT_SetTestNumber(2);
    UT_TestInfo("Step with max positive PWM for 1 second");
    motor_simulation_step(1000, 1.0);
    double v1 = motor_get_pot_voltage();
    UT_CheckTrue("Voltage increases after max PWM step", v1 > MOTOR_POT_MIN);

    UT_SetTestNumber(3);
    UT_TestInfo("Step with max negative PWM for 1 second (should go back to min)");
    motor_simulation_step(-1000, 1.0);
    double v2 = motor_get_pot_voltage();
    UT_CheckInRange((int)(v2 * 1000), (int)(MOTOR_POT_MIN * 1000), 55, "Voltage returns to min after negative PWM");

    UT_SetTestNumber(4);
    UT_TestInfo("Step with zero PWM (should not change)");
    double v_before = motor_get_pot_voltage();
    motor_simulation_step(0, 1.0);
    double v_after = motor_get_pot_voltage();
    UT_CheckInRange((int)(v_after * 1000), (int)(v_before * 1000), 55, "Voltage unchanged with zero PWM");

    UT_SetTestNumber(5);
    UT_TestInfo("Motor moves at correct speed (positive direction)");
    motor_simulation_reset();
    double dt = 0.5;
    motor_simulation_step(1000, dt);
    double v_expected = MOTOR_POT_MIN + MOTOR_MAX_SPEED_V_PER_S * dt;
    double v_actual = motor_get_pot_voltage();
    UT_CheckInRange((int)(v_actual * 1000), (int)(v_expected * 1000), 55, "Motor moves at correct speed (positive)");

    UT_SetTestNumber(6);
    UT_TestInfo("Motor saturates at max voltage");
    motor_simulation_reset();
    motor_simulation_step(1000, 10.0);
    double v_max = motor_get_pot_voltage();
    UT_CheckInRange((int)(v_max * 1000), (int)(MOTOR_POT_MAX * 1000), 55, "Motor saturates at max voltage");

    UT_SetTestNumber(7);
    UT_TestInfo("Motor saturates at min voltage");
    motor_simulation_reset();
    motor_simulation_step(1000, 10.0);
    motor_simulation_step(-1000, 20.0);
    double v_min = motor_get_pot_voltage();
    UT_CheckInRange((int)(v_min * 1000), (int)(MOTOR_POT_MIN * 1000), 55, "Motor saturates at min voltage");

    std::cout << "Motor simulation tests complete.\n";
}