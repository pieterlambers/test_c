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
    MotorController ctrl(MOTOR_MEAS_NOISE_V, 1000, 150, 0.5);

    // Register PWM callback to drive the simulation
    ctrl.register_pwm_callback([](int pwm) {
        motor_simulation_step(pwm, 0.01);
    });

    // Check initial state is Initialising
    UT_CheckTrue("Controller starts in Initialising state", ctrl.get_state() == MotorControllerState::Initialising);

    // Start calibration (hold 0.2s at each end)
    ctrl.start_calibration(0.2);

    // Simulate until calibration is done or timeout
    double sim_time = 0.0;
    double dt = 0.01;
    bool calibrated = false;
    bool was_busy = false;
    for (; sim_time < 10.0; sim_time += dt) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt);
        if (ctrl.get_state() == MotorControllerState::Busy) was_busy = true;
        if (ctrl.get_state() == MotorControllerState::Idle) {
            calibrated = true;
            break;
        }
    }
    UT_CheckTrue("Calibration completes and controller becomes idle", calibrated);
    UT_CheckTrue("Controller was busy during calibration", was_busy);

    // Use a tolerance that matches the controller's deadband (hysteresis)
    double tol = 2 * MOTOR_MEAS_NOISE_V + 0.01 + 0.005;

    // Check that calibrated min/max are close to simulation limits
    UT_CheckInRange(ctrl.get_pot_min(), MOTOR_POT_MIN, tol, "Calibrated min close to MOTOR_POT_MIN");
    UT_CheckInRange(ctrl.get_pot_max(), MOTOR_POT_MAX, tol, "Calibrated max close to MOTOR_POT_MAX");

    // --- Test 2: Regulation to a position ---
    UT_SetTestNumber(2);
    UT_TestInfo("Motor regulates to 50% position");

    ctrl.set_target_percent(50.0);

    bool was_busy_reg = false;
    for (int i = 0; i < 200; ++i) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt);
        if (ctrl.get_state() == MotorControllerState::Busy) was_busy_reg = true;
    }
    double v_target = ctrl.get_pot_min() + 0.5 * (ctrl.get_pot_max() - ctrl.get_pot_min());
    double v_actual = motor_get_pot_voltage();
    UT_CheckInRange(v_actual, v_target, tol, "Motor reaches 50% position");
    UT_CheckTrue("Controller was busy during regulation to 50%", was_busy_reg);
    UT_CheckTrue("Controller is idle at 50% target", ctrl.get_state() == MotorControllerState::Idle);

    // --- Test 3: Regulation to max position ---
    UT_SetTestNumber(3);
    UT_TestInfo("Motor regulates to 100% position");

    ctrl.set_target_percent(100.0);
    was_busy_reg = false;
    for (int i = 0; i < 200; ++i) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt);
        if (ctrl.get_state() == MotorControllerState::Busy) was_busy_reg = true;
    }
    v_target = ctrl.get_pot_max();
    v_actual = motor_get_pot_voltage();
    UT_CheckInRange(v_actual, v_target, tol, "Motor reaches max position");
    UT_CheckTrue("Controller was busy during regulation to max", was_busy_reg);
    UT_CheckTrue("Controller is idle at max target", ctrl.get_state() == MotorControllerState::Idle);

    // --- Test 4: Regulation to min position ---
    UT_SetTestNumber(4);
    UT_TestInfo("Motor regulates to 0% position");

    ctrl.set_target_percent(0.0);
    was_busy_reg = false;
    for (int i = 0; i < 200; ++i) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt);
        if (ctrl.get_state() == MotorControllerState::Busy) was_busy_reg = true;
    }
    v_target = ctrl.get_pot_min();
    v_actual = motor_get_pot_voltage();
    UT_CheckInRange(v_actual, v_target, tol, "Motor reaches min position");
    UT_CheckTrue("Controller was busy during regulation to min", was_busy_reg);
    UT_CheckTrue("Controller is idle at min target", ctrl.get_state() == MotorControllerState::Idle);

    // --- Test 5: PWM ramps down as error decreases ---
    UT_SetTestNumber(5);
    UT_TestInfo("PWM output ramps down as error decreases");

    ctrl.set_target_percent(0.0);
    for (int i = 0; i < 200; ++i) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt);
    }

    ctrl.set_target_percent(80.0);

    int last_pwm = 0;
    int pwm_val = 0;
    bool ramped_down = false;

    ctrl.register_pwm_callback([&pwm_val](int pwm) {
        pwm_val = pwm;
        motor_simulation_step(pwm, 0.01);
    });

    std::cout << "Step\tPWM\n";
    for (int i = 0; i < 200; ++i) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt);
        std::cout << i << "\t" << pwm_val << "\n";
        if (i > 0 && std::abs(pwm_val) < std::abs(last_pwm) && std::abs(pwm_val) > 0) {
            ramped_down = true;
        }
        last_pwm = pwm_val;
    }
    UT_CheckTrue("PWM ramps down as error decreases", ramped_down);

    // --- Test 6: Detect overshoot and oscillation (should be none) ---
    UT_SetTestNumber(6);
    UT_TestInfo("Motor does not overshoot or oscillate near the target position");

    ctrl.set_target_percent(0.0);
    for (int i = 0; i < 200; ++i) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt);
    }

    ctrl.set_target_percent(100.0);

    bool overshoot_detected = false;
    bool oscillation_detected = false;
    double max_target = ctrl.get_pot_max();
    int last_pwm6 = 0;
    int pwm_val6 = 0;

    ctrl.register_pwm_callback([&pwm_val6](int pwm) {
        pwm_val6 = pwm;
        motor_simulation_step(pwm, 0.01);
    });

    for (int i = 0; i < 200; ++i) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt);
        double v = motor_get_pot_voltage();
        if (v > max_target + tol) {
            overshoot_detected = true;
            break;
        }
        if (i > 0 && (pwm_val6 * last_pwm6 < 0)) {
            oscillation_detected = true;
        }
        last_pwm6 = pwm_val6;
    }
    UT_CheckTrue("No overshoot beyond max target", !overshoot_detected);
    UT_CheckTrue("No PWM oscillation near target", !oscillation_detected);

    std::cout << "Motor control tests complete.\n";
}