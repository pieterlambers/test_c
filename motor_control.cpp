#include "motor_control.h"

MotorController::MotorController()
    : calib_state_(CalibState::NotCalibrated),
      public_state_(MotorControllerState::Initialising),
      hold_time_s_(1.0),
      hold_timer_(0.0),
      target_percent_(0.0),
      measured_voltage_(0.0),
      pwm_cb_(nullptr),
      hysteresis_(0.01),
      pot_min_(0.0),
      pot_max_(1.0) // default, will be set by calibration
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
}

void MotorController::update(double dt_s) {
    const double stable_threshold = 0.001; // Volts, adjust as needed

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
            // Use calibrated min/max for control
            double target_voltage = pot_min_ + (target_percent_ / 100.0) * (pot_max_ - pot_min_);
            int pwm = 0;
            if (measured_voltage_ < target_voltage - hysteresis_) {
                pwm = 1000;
                public_state_ = MotorControllerState::Busy;
            } else if (measured_voltage_ > target_voltage + hysteresis_) {
                pwm = -1000;
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