#include <iostream>
#include "unit_test.h"
#include "motor_simulation.h"
#include "motor_control.h"

// Helper: Simulate time steps and update both simulation and controller
void simulate_motor(MotorController& ctrl, double duration_s, double dt_s) {
    for (double t = 0; t < duration_s; t += dt_s) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt_s);
        // The PWM callback will call motor_simulation_step()
    }
}

void Run_MotorControl_Tests() {
    UT_SetLogFile("motor_control_testresults.txt");
    std::cout << "Motor Control Unit Tests\n";

    // --- Test 1: Calibration ---
    UT_SetTestNumber(1);
    UT_TestInfo("Motor calibrates to max and min positions");

    motor_simulation_reset();
    // Pass the noise level from the simulation to the controller
    MotorController ctrl(MOTOR_MEAS_NOISE_V);

    // Register PWM callback to drive the simulation
    ctrl.register_pwm_callback([](int pwm) {
        // Simulate with 10ms steps
        motor_simulation_step(pwm, 0.01);
    });

    // Start calibration (hold 0.2s at each end)
    ctrl.start_calibration(0.2);

    // Simulate until calibration is done or timeout
    double sim_time = 0.0;
    double dt = 0.01;
    bool calibrated = false;
    for (; sim_time < 10.0; sim_time += dt) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt);
        if (ctrl.get_state() == MotorControllerState::Idle) {
            calibrated = true;
            break;
        }
    }
    UT_CheckTrue("Calibration completes and controller becomes idle", calibrated);

    // Use a tolerance slightly above the noise amplitude (in mV)
    int tol = static_cast<int>(MOTOR_MEAS_NOISE_V * 1000) + 5;

    // Check that calibrated min/max are close to simulation limits
    UT_CheckInRange((int)(ctrl.get_pot_min() * 1000), (int)(MOTOR_POT_MIN * 1000), tol, "Calibrated min close to MOTOR_POT_MIN");
    UT_CheckInRange((int)(ctrl.get_pot_max() * 1000), (int)(MOTOR_POT_MAX * 1000), tol, "Calibrated max close to MOTOR_POT_MAX");

    // --- Test 2: Regulation to a position ---
    UT_SetTestNumber(2);
    UT_TestInfo("Motor regulates to 50% position");

    ctrl.set_target_percent(50.0);

    for (int i = 0; i < 200; ++i) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt);
    }

    double v_target = ctrl.get_pot_min() + 0.5 * (ctrl.get_pot_max() - ctrl.get_pot_min());
    double v_actual = motor_get_pot_voltage();
    UT_CheckInRange((int)(v_actual * 1000), (int)(v_target * 1000), tol, "Motor reaches 50% position");

    // --- Test 3: Regulation to max position ---
    UT_SetTestNumber(3);
    UT_TestInfo("Motor regulates to 100% position");

    ctrl.set_target_percent(100.0);
    for (int i = 0; i < 200; ++i) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt);
    }
    v_target = ctrl.get_pot_max();
    v_actual = motor_get_pot_voltage();
    UT_CheckInRange((int)(v_actual * 1000), (int)(v_target * 1000), tol, "Motor reaches max position");

    // --- Test 4: Regulation to min position ---
    UT_SetTestNumber(4);
    UT_TestInfo("Motor regulates to 0% position");

    ctrl.set_target_percent(0.0);
    for (int i = 0; i < 200; ++i) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt);
    }
    v_target = ctrl.get_pot_min();
    v_actual = motor_get_pot_voltage();
    UT_CheckInRange((int)(v_actual * 1000), (int)(v_target * 1000), tol, "Motor reaches min position");

    std::cout << "Motor control tests complete.\n";
}