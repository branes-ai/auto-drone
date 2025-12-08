#include "PIDController.hpp"
#include <cmath>

namespace control_algorithms {

// --- PIDController ---

PIDController::PIDController(const Config& config)
    : config_(config) {}

PIDController::PIDController(const Gains& gains) {
    config_.gains = gains;
}

PIDController::PIDController(float kp, float ki, float kd) {
    config_.gains = {kp, ki, kd};
}

void PIDController::set_config(const Config& config) {
    config_ = config;
}

void PIDController::set_gains(const Gains& gains) {
    config_.gains = gains;
}

void PIDController::set_gains(float kp, float ki, float kd) {
    config_.gains = {kp, ki, kd};
}

void PIDController::set_limits(const Limits& limits) {
    config_.limits = limits;
}

float PIDController::update(float dt, float setpoint, float measurement) {
    return update(dt, setpoint, measurement, 0.0f);
}

float PIDController::update(float dt, float setpoint, float measurement, float feedforward) {
    if (dt <= 0.0f) {
        return 0.0f;
    }

    float error = setpoint - measurement;

    // Proportional term
    p_term_ = config_.gains.kp * error;

    // Integral term with anti-windup
    integral_ += error * dt;
    integral_ = std::clamp(integral_, config_.limits.integral_min, config_.limits.integral_max);
    i_term_ = config_.gains.ki * integral_;

    // Derivative term with low-pass filter
    if (first_update_) {
        filtered_derivative_ = 0.0f;
        first_update_ = false;
    } else {
        float raw_derivative = (error - last_error_) / dt;
        // First-order low-pass filter: y = alpha * x + (1 - alpha) * y_prev
        // alpha = dt / (tau + dt)
        float alpha = dt / (config_.derivative_filter_tau + dt);
        filtered_derivative_ = alpha * raw_derivative + (1.0f - alpha) * filtered_derivative_;
    }
    d_term_ = config_.gains.kd * filtered_derivative_;

    last_error_ = error;

    // Sum terms and apply output limits
    float output = p_term_ + i_term_ + d_term_ + feedforward;
    output = std::clamp(output, config_.limits.output_min, config_.limits.output_max);

    return output;
}

void PIDController::reset() {
    integral_ = 0.0f;
    last_error_ = 0.0f;
    filtered_derivative_ = 0.0f;
    first_update_ = true;
    p_term_ = 0.0f;
    i_term_ = 0.0f;
    d_term_ = 0.0f;
}

// --- PositionController ---

PositionController::PositionController(const Config& config)
    : pid_x_(config.x), pid_y_(config.y), pid_z_(config.z) {}

PositionController::PositionController(float kp, float ki, float kd)
    : pid_x_(kp, ki, kd), pid_y_(kp, ki, kd), pid_z_(kp, ki, kd) {}

void PositionController::set_config(const Config& config) {
    pid_x_.set_config(config.x);
    pid_y_.set_config(config.y);
    pid_z_.set_config(config.z);
}

void PositionController::update(float dt,
                                 float target_x, float target_y, float target_z,
                                 float current_x, float current_y, float current_z,
                                 float& out_vx, float& out_vy, float& out_vz) {
    out_vx = pid_x_.update(dt, target_x, current_x);
    out_vy = pid_y_.update(dt, target_y, current_y);
    out_vz = pid_z_.update(dt, target_z, current_z);
}

void PositionController::reset() {
    pid_x_.reset();
    pid_y_.reset();
    pid_z_.reset();
}

// --- YawController ---

YawController::YawController(const PIDController::Config& config)
    : pid_(config) {}

YawController::YawController(float kp, float ki, float kd)
    : pid_(kp, ki, kd) {}

void YawController::set_config(const PIDController::Config& config) {
    pid_.set_config(config);
}

float YawController::update(float dt, float target_yaw, float current_yaw) {
    // Compute error with angle wrapping
    float error = wrap_angle(target_yaw - current_yaw);

    // Use the PID controller but pass current as 0 and setpoint as error
    // This way the PID sees the wrapped error directly
    return pid_.update(dt, error, 0.0f);
}

void YawController::reset() {
    pid_.reset();
}

float YawController::wrap_angle(float angle) {
    // Wrap angle to [-PI, PI]
    constexpr float PI = 3.14159265358979323846f;
    constexpr float TWO_PI = 2.0f * PI;

    while (angle > PI) {
        angle -= TWO_PI;
    }
    while (angle < -PI) {
        angle += TWO_PI;
    }
    return angle;
}

} // namespace control_algorithms
