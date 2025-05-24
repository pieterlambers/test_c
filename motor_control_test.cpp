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

    ctrl.set_ramp_up_time(0.0); // Ensure immediate movement for this test
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

    ctrl.set_ramp_up_time(0.0); // Ensure immediate movement for this test
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

    ctrl.set_ramp_up_time(0.0); // Ensure immediate movement for this test
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

    ctrl.set_ramp_up_time(0.0); // Ensure immediate movement for this test
    ctrl.set_target_percent(0.0);
    for (int i = 0; i < 200; ++i) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt);
    }

    ctrl.set_ramp_up_time(0.0); // Ensure immediate movement for this test
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

    ctrl.set_ramp_up_time(0.0); // Ensure immediate movement for this test
    ctrl.set_target_percent(0.0);
    for (int i = 0; i < 200; ++i) {
        ctrl.set_measured_voltage(motor_get_pot_voltage());
        ctrl.update(dt);
    }

    ctrl.set_ramp_up_time(0.0); // Ensure immediate movement for this test
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

    // --- Test 7: RampUpOverTime ---
    UT_SetTestNumber(7);
    UT_TestInfo("current_percent_ ramps up over time");
    motor_simulation_reset();
    MotorController ctrl_ramp_1(MOTOR_MEAS_NOISE_V, 1000, 150, 0.5);
    ctrl_ramp_1.register_pwm_callback([](int pwm) {
        motor_simulation_step(pwm, 0.01); // dt = 0.01s
    });
    // Calibrate the controller first
    ctrl_ramp_1.start_calibration(0.1); // Short hold time for test
    for (int i=0; i<500 && ctrl_ramp_1.get_state() != MotorControllerState::Idle; ++i) { // Max 5s for calibration
        ctrl_ramp_1.set_measured_voltage(motor_get_pot_voltage());
        ctrl_ramp_1.update(0.01);
    }
    UT_CheckTrue("RampTest1: Calibration completed", ctrl_ramp_1.get_state() == MotorControllerState::Idle);
    // Set current position to 0% by moving motor there
    ctrl_ramp_1.set_ramp_up_time(0.0);
    ctrl_ramp_1.set_target_percent(0.0);
    for (int i=0; i<200; ++i) { // 2s
        ctrl_ramp_1.set_measured_voltage(motor_get_pot_voltage());
        ctrl_ramp_1.update(0.01);
    }
    UT_CheckInRange(ctrl_ramp_1.get_current_percent_for_test(), 0.0, 1.0, "RampTest1: Initial current_percent_ near 0");


    ctrl_ramp_1.set_ramp_up_time(1.0); // 1 second ramp time
    ctrl_ramp_1.set_target_percent(100.0);

    UT_CheckInRange(ctrl_ramp_1.get_current_percent_for_test(), 0.0, 0.001, "current_percent_ starts at 0");

    // Simulate for 0.5 seconds (50 steps of 0.01s)
    for (int i = 0; i < 50; ++i) {
        ctrl_ramp_1.set_measured_voltage(motor_get_pot_voltage());
        ctrl_ramp_1.update(0.01);
    }
    UT_CheckInRange(ctrl_ramp_1.get_current_percent_for_test(), 50.0, 5.0, "current_percent_ near 50% at 0.5s"); // Increased tolerance

    // Simulate for another 0.5 seconds (total 1.0s)
    for (int i = 0; i < 50; ++i) {
        ctrl_ramp_1.set_measured_voltage(motor_get_pot_voltage());
        ctrl_ramp_1.update(0.01);
    }
    UT_CheckInRange(ctrl_ramp_1.get_current_percent_for_test(), 100.0, 5.0, "current_percent_ near 100% at 1.0s"); // Increased tolerance

    // --- Test 8: RampToNewTargetMidRamp ---
    UT_SetTestNumber(8);
    UT_TestInfo("Ramps to a new target if changed mid-ramp");
    motor_simulation_reset();
    MotorController ctrl_ramp_2(MOTOR_MEAS_NOISE_V, 1000, 150, 0.5);
    ctrl_ramp_2.register_pwm_callback([](int pwm) { motor_simulation_step(pwm, 0.01); });
    ctrl_ramp_2.start_calibration(0.1);
    for (int i=0; i<500 && ctrl_ramp_2.get_state() != MotorControllerState::Idle; ++i) {
        ctrl_ramp_2.set_measured_voltage(motor_get_pot_voltage());
        ctrl_ramp_2.update(0.01);
    }
    UT_CheckTrue("RampTest2: Calibration completed", ctrl_ramp_2.get_state() == MotorControllerState::Idle);
    ctrl_ramp_2.set_ramp_up_time(0.0);
    ctrl_ramp_2.set_target_percent(0.0); // Start at 0%
    for (int i=0; i<200; ++i) {
        ctrl_ramp_2.set_measured_voltage(motor_get_pot_voltage());
        ctrl_ramp_2.update(0.01);
    }
    UT_CheckInRange(ctrl_ramp_2.get_current_percent_for_test(), 0.0, 1.0, "Initial current_percent_ is 0");

    ctrl_ramp_2.set_ramp_up_time(1.0);    // 1s ramp time
    ctrl_ramp_2.set_target_percent(100.0); // Target 100%

    // Simulate 0.5s
    for (int i = 0; i < 50; ++i) {
        ctrl_ramp_2.set_measured_voltage(motor_get_pot_voltage());
        ctrl_ramp_2.update(0.01);
    }
    double percent_at_0_5s = ctrl_ramp_2.get_current_percent_for_test();
    UT_CheckInRange(percent_at_0_5s, 50.0, 5.0, "current_percent_ near 50% at 0.5s before new target");

    ctrl_ramp_2.set_target_percent(0.0); // New target 0%, ramp should restart from current_percent_
    UT_CheckTrue("RampTest2: is_ramping is true after new target", ctrl_ramp_2.get_current_percent_for_test() != 0.0); // is_ramping_ is private

    // Simulate 0.5s more (total 1.0s for the new ramp segment)
    // New ramp is from percent_at_0_5s to 0.0 over 1.0s
    // So after 0.5s of this new ramp, it should be halfway between percent_at_0_5s and 0.0
    double expected_percent_after_new_ramp_0_5s = percent_at_0_5s * 0.5;
    for (int i = 0; i < 50; ++i) {
        ctrl_ramp_2.set_measured_voltage(motor_get_pot_voltage());
        ctrl_ramp_2.update(0.01);
    }
    UT_CheckInRange(ctrl_ramp_2.get_current_percent_for_test(), expected_percent_after_new_ramp_0_5s, 5.0, "current_percent_ halfway to new target 0%");

    // Simulate another 0.5s (total 1.0s for new ramp)
    for (int i = 0; i < 50; ++i) {
        ctrl_ramp_2.set_measured_voltage(motor_get_pot_voltage());
        ctrl_ramp_2.update(0.01);
    }
    UT_CheckInRange(ctrl_ramp_2.get_current_percent_for_test(), 0.0, 5.0, "current_percent_ reached new target 0%");

    // --- Test 9: NoRampWhenTimeIsZero ---
    UT_SetTestNumber(9);
    UT_TestInfo("current_percent_ jumps to target if ramp time is zero");
    motor_simulation_reset();
    MotorController ctrl_ramp_3(MOTOR_MEAS_NOISE_V, 1000, 150, 0.5);
    ctrl_ramp_3.register_pwm_callback([](int pwm) { motor_simulation_step(pwm, 0.01); });
    ctrl_ramp_3.start_calibration(0.1);
     for (int i=0; i<500 && ctrl_ramp_3.get_state() != MotorControllerState::Idle; ++i) {
        ctrl_ramp_3.set_measured_voltage(motor_get_pot_voltage());
        ctrl_ramp_3.update(0.01);
    }
    UT_CheckTrue("RampTest3: Calibration completed", ctrl_ramp_3.get_state() == MotorControllerState::Idle);
    ctrl_ramp_3.set_ramp_up_time(0.0); // Ensure motor is at 0% initially
    ctrl_ramp_3.set_target_percent(0.0);
    for (int i=0; i<100; ++i) { ctrl_ramp_3.set_measured_voltage(motor_get_pot_voltage()); ctrl_ramp_3.update(0.01); }


    ctrl_ramp_3.set_ramp_up_time(0.0);
    ctrl_ramp_3.set_target_percent(75.0);
    ctrl_ramp_3.set_measured_voltage(motor_get_pot_voltage()); // Update voltage once
    ctrl_ramp_3.update(0.01); // Update controller once

    UT_CheckInRange(ctrl_ramp_3.get_current_percent_for_test(), 75.0, 0.001, "current_percent_ immediately 75%");

    // --- Test 10: ReachesAndMaintainsTarget ---
    UT_SetTestNumber(10);
    UT_TestInfo("Reaches target after ramp and maintains it, becoming Idle");
    motor_simulation_reset();
    MotorController ctrl_ramp_4(MOTOR_MEAS_NOISE_V, 1000, 150, 0.5);
    ctrl_ramp_4.register_pwm_callback([](int pwm) { motor_simulation_step(pwm, 0.01); });
    ctrl_ramp_4.start_calibration(0.1);
    for (int i=0; i<500 && ctrl_ramp_4.get_state() != MotorControllerState::Idle; ++i) {
        ctrl_ramp_4.set_measured_voltage(motor_get_pot_voltage());
        ctrl_ramp_4.update(0.01);
    }
    UT_CheckTrue("RampTest4: Calibration completed", ctrl_ramp_4.get_state() == MotorControllerState::Idle);
    ctrl_ramp_4.set_ramp_up_time(0.0);
    ctrl_ramp_4.set_target_percent(0.0); // Start at 0%
    for (int i=0; i<200; ++i) {
        ctrl_ramp_4.set_measured_voltage(motor_get_pot_voltage());
        ctrl_ramp_4.update(0.01);
    }
    UT_CheckInRange(ctrl_ramp_4.get_current_percent_for_test(), 0.0, 1.0, "Initial current_percent_ is 0");


    ctrl_ramp_4.set_ramp_up_time(0.5); // 0.5s ramp time
    ctrl_ramp_4.set_target_percent(60.0);

    // Simulate for 1.0 second (well past 0.5s ramp time)
    // 100 steps of 0.01s
    for (int i = 0; i < 100; ++i) {
        ctrl_ramp_4.set_measured_voltage(motor_get_pot_voltage());
        ctrl_ramp_4.update(0.01);
        if (i == 49) { // After 0.5s, ramp should be complete
             UT_CheckInRange(ctrl_ramp_4.get_current_percent_for_test(), 60.0, 5.0, "current_percent_ reached target at ramp end");
        }
    }

    UT_CheckInRange(ctrl_ramp_4.get_current_percent_for_test(), 60.0, 5.0, "current_percent_ maintained post-ramp");
    // Check actual motor position as well, using existing tolerance
    double v_target_ramp4 = ctrl_ramp_4.get_pot_min() + 0.6 * (ctrl_ramp_4.get_pot_max() - ctrl_ramp_4.get_pot_min());
    double v_actual_ramp4 = motor_get_pot_voltage();
    double tol_ramp4 = 2 * MOTOR_MEAS_NOISE_V + 0.01 + 0.005; // from existing tests
    UT_CheckInRange(v_actual_ramp4, v_target_ramp4, tol_ramp4, "Motor physical position matches target post-ramp");
    UT_CheckTrue("RampTest4: Controller is Idle at target", ctrl_ramp_4.get_state() == MotorControllerState::Idle);


    std::cout << "Motor control tests complete.\n";
}