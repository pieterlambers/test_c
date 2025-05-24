#pragma once
#include <functional>

enum class MotorControllerState {
    Initialising,
    Idle,
    Busy
};

class MotorController {
public:
    // Add noise_level as a parameter with a default value
    explicit MotorController(double noise_level = 0.0);

    void start_calibration(double hold_time_s);
    void update(double dt_s);

    // Set the target position in percent (0..100)
    void set_target_percent(double percent);

    void set_measured_voltage(double volts);
    void register_pwm_callback(std::function<void(int pwm)> cb);

    MotorControllerState get_state() const;

    // Access calibrated min/max values
    double get_pot_min() const { return pot_min_; }
    double get_pot_max() const { return pot_max_; }

private:
    enum class CalibState {
        NotCalibrated,
        MovingToMax,
        AtMax,
        MovingToMin,
        AtMin,
        Calibrated
    };

    CalibState calib_state_;
    MotorControllerState public_state_;

    double hold_time_s_;
    double hold_timer_;
    double target_percent_; // 0..100
    double measured_voltage_;
    std::function<void(int pwm)> pwm_cb_;
    double hysteresis_;

    // Calibrated min/max values
    double pot_min_;
    double pot_max_;
    double prev_measured_voltage_;
    double stable_timer_;
    double noise_level_; // Noise amplitude (V) for stability threshold
};