#include "motor_control.h"
#include <cmath>

MotorController::MotorController(double noise_level, int pwm_max, int pwm_min, double error_full)
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
      error_full_(error_full)
{}

void MotorController::set_measured_voltage(double volts) {
    measured_voltage_ = volts;
}

void MotorController::set_target_percent(double percent) {
    if (percent < 0.0) percent = 0.0;
    if (percent > 100.0) percent = 100.0;
    target_percent_ = percent;
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
            double target_voltage = pot_min_ + (target_percent_ / 100.0) * (pot_max_ - pot_min_);
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