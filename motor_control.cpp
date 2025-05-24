#include "motor_control.h"
#include <cmath>
#include <algorithm> // Required for std::min

MotorController::MotorController(double noise_level, int pwm_max, int pwm_min, double error_full, double default_ramp_up_time_s)
    : calib_state_(CalibState::NotCalibrated),
      public_state_(MotorControllerState::Initialising),
      hold_time_s_(1.0),
      hold_timer_(0.0),
      target_percent_(0.0),
      measured_voltage_(0.0),
      pwm_cb_(nullptr),
      hysteresis_(2 * noise_level + 0.01),
      pot_min_(0.0),
      pot_max_(1.0),
      prev_measured_voltage_(0.0),
      stable_timer_(0.0),
      noise_level_(noise_level),
      pwm_max_(pwm_max),
      pwm_min_(pwm_min),
      error_full_(error_full),
      current_percent_(0.0),
      final_target_percent_(0.0),
      initial_percent_for_ramp_(0.0),
      ramp_up_time_s_(default_ramp_up_time_s), // Initialize with the new parameter
      ramp_timer_s_(0.0),
      is_ramping_(false)
{
    // Ensure ramp_up_time_s_ is not negative from the default value
    if (ramp_up_time_s_ < 0.0) {
        ramp_up_time_s_ = 0.0;
    }
}

void MotorController::set_measured_voltage(double volts) {
    measured_voltage_ = volts;
}

void MotorController::set_ramp_up_time(double time_s) {
    if (time_s < 0.0) {
        ramp_up_time_s_ = 0.0;
    } else {
        ramp_up_time_s_ = time_s;
    }
}

void MotorController::set_target_percent(double percent) {
    if (percent < 0.0) percent = 0.0;
    if (percent > 100.0) percent = 100.0;

    final_target_percent_ = percent;
    initial_percent_for_ramp_ = current_percent_;

    if (ramp_up_time_s_ > 0.0 && final_target_percent_ != current_percent_) {
        is_ramping_ = true;
        ramp_timer_s_ = 0.0;
    } else {
        current_percent_ = final_target_percent_;
        is_ramping_ = false;
    }
    // target_percent_ should now be current_percent_ in the update loop for PID
    // However, the original PID logic uses target_percent_ directly.
    // For now, let's keep target_percent_ as the final target for PID,
    // and current_percent_ will be updated by the ramping logic in update()
    // This might need further adjustment depending on how update() is modified.
    target_percent_ = final_target_percent_; // Keep this for now, may need review
}

void MotorController::register_pwm_callback(std::function<void(int pwm)> cb) {
    pwm_cb_ = cb;
}

MotorControllerState MotorController::get_state() const {
    return public_state_;
}

void MotorController::start_calibration(double hold_time_s) {
    calib_state_ = CalibState::MovingToMax;
    public_state_ = MotorControllerState::Busy;
    hold_time_s_ = hold_time_s;
    hold_timer_ = 0.0;
    // Optionally reset min/max
    pot_min_ = 0.0;
    pot_max_ = 1.0;
    stable_timer_ = 0.0;
    prev_measured_voltage_ = measured_voltage_;
}

void MotorController::update(double dt_s) {
    const double stable_threshold = noise_level_ * 1.2;

    switch (calib_state_) {
        case CalibState::NotCalibrated:
            public_state_ = MotorControllerState::Initialising;
            break;
        case CalibState::MovingToMax:
            public_state_ = MotorControllerState::Busy;
            if (pwm_cb_) pwm_cb_(1000);
            if (std::abs(measured_voltage_ - prev_measured_voltage_) < stable_threshold) {
                stable_timer_ += dt_s;
                if (stable_timer_ >= hold_time_s_) {
                    pot_max_ = measured_voltage_;
                    calib_state_ = CalibState::MovingToMin;
                    stable_timer_ = 0.0;
                }
            } else {
                stable_timer_ = 0.0;
            }
            break;
        case CalibState::MovingToMin:
            public_state_ = MotorControllerState::Busy;
            if (pwm_cb_) pwm_cb_(-1000);
            if (std::abs(measured_voltage_ - prev_measured_voltage_) < stable_threshold) {
                stable_timer_ += dt_s;
                if (stable_timer_ >= hold_time_s_) {
                    pot_min_ = measured_voltage_;
                    calib_state_ = CalibState::Calibrated;
                    if (pwm_cb_) pwm_cb_(0);
                }
            } else {
                stable_timer_ = 0.0;
            }
            break;
        case CalibState::Calibrated: {
            // Ramping logic
            if (is_ramping_) {
                ramp_timer_s_ += dt_s;
                if (ramp_timer_s_ >= ramp_up_time_s_) {
                    current_percent_ = final_target_percent_;
                    is_ramping_ = false;
                    ramp_timer_s_ = ramp_up_time_s_; // Cap timer
                } else {
                    double ramp_progress = ramp_timer_s_ / ramp_up_time_s_;
                    current_percent_ = initial_percent_for_ramp_ + (final_target_percent_ - initial_percent_for_ramp_) * ramp_progress;
                }
            }

            double actual_target_percent_for_pid = current_percent_;
            // If not ramping, current_percent_ should be equal to final_target_percent_ (which is target_percent_)
            // If ramping, current_percent_ is the intermediate target.

            double target_voltage = pot_min_ + (actual_target_percent_for_pid / 100.0) * (pot_max_ - pot_min_);
            double error = measured_voltage_ - target_voltage;
            int pwm = 0;

            // Use member variables for scaling
            if (error < -hysteresis_) {
                double scale = std::min(1.0, std::abs(error) / error_full_);
                pwm = static_cast<int>(pwm_min_ + (pwm_max_ - pwm_min_) * scale);
                public_state_ = MotorControllerState::Busy;
            } else if (error > hysteresis_) {
                double scale = std::min(1.0, std::abs(error) / error_full_);
                pwm = -static_cast<int>(pwm_min_ + (pwm_max_ - pwm_min_) * scale);
                public_state_ = MotorControllerState::Busy;
            } else {
                pwm = 0;
                public_state_ = MotorControllerState::Idle;
            }
            if (pwm_cb_) pwm_cb_(pwm);
            break;
        }
        default:
            break;
    }
    prev_measured_voltage_ = measured_voltage_;
}