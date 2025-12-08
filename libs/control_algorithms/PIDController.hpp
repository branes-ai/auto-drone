#pragma once

#include <chrono>
#include <algorithm>
#include <cmath>

namespace control_algorithms {

// Generic PID controller with anti-windup and derivative filtering
// Suitable for position, velocity, or attitude control loops
class PIDController {
public:
    struct Gains {
        float kp = 1.0f;   // Proportional gain
        float ki = 0.0f;   // Integral gain
        float kd = 0.0f;   // Derivative gain
    };

    struct Limits {
        float output_min = -1.0f;      // Minimum output value
        float output_max = 1.0f;       // Maximum output value
        float integral_min = -10.0f;   // Anti-windup: min integral accumulator
        float integral_max = 10.0f;    // Anti-windup: max integral accumulator
    };

    struct Config {
        Gains gains;
        Limits limits;
        float derivative_filter_tau = 0.02f;  // Low-pass filter time constant for derivative (seconds)
    };

    PIDController() = default;
    explicit PIDController(const Config& config);
    explicit PIDController(const Gains& gains);
    PIDController(float kp, float ki, float kd);

    // Set configuration
    void set_config(const Config& config);
    void set_gains(const Gains& gains);
    void set_gains(float kp, float ki, float kd);
    void set_limits(const Limits& limits);

    // Get current configuration
    const Config& config() const { return config_; }

    // Compute control output
    // dt: time step in seconds
    // setpoint: desired value
    // measurement: current measured value
    // Returns: control output (clamped to limits)
    float update(float dt, float setpoint, float measurement);

    // Compute control output with feedforward term
    float update(float dt, float setpoint, float measurement, float feedforward);

    // Reset controller state (integral accumulator and derivative filter)
    void reset();

    // Get internal state for debugging/telemetry
    float error() const { return last_error_; }
    float integral() const { return integral_; }
    float derivative() const { return filtered_derivative_; }
    float p_term() const { return p_term_; }
    float i_term() const { return i_term_; }
    float d_term() const { return d_term_; }

private:
    Config config_;

    // State
    float integral_ = 0.0f;
    float last_error_ = 0.0f;
    float filtered_derivative_ = 0.0f;
    bool first_update_ = true;

    // Debug values from last update
    float p_term_ = 0.0f;
    float i_term_ = 0.0f;
    float d_term_ = 0.0f;
};

// 3D position controller using three independent PID loops
// Outputs velocity commands (vx, vy, vz) to track a target position
class PositionController {
public:
    struct Config {
        PIDController::Config x;
        PIDController::Config y;
        PIDController::Config z;
    };

    PositionController() = default;
    explicit PositionController(const Config& config);

    // Convenience constructor with same gains for all axes
    PositionController(float kp, float ki, float kd);

    void set_config(const Config& config);

    // Compute velocity commands to reach target position
    // dt: time step in seconds
    // target_x/y/z: desired position
    // current_x/y/z: current position
    // out_vx/vy/vz: output velocity commands
    void update(float dt,
                float target_x, float target_y, float target_z,
                float current_x, float current_y, float current_z,
                float& out_vx, float& out_vy, float& out_vz);

    void reset();

    // Access individual axis controllers
    PIDController& x() { return pid_x_; }
    PIDController& y() { return pid_y_; }
    PIDController& z() { return pid_z_; }

private:
    PIDController pid_x_;
    PIDController pid_y_;
    PIDController pid_z_;
};

// Yaw controller with angle wrapping
// Handles the discontinuity at +/- PI
class YawController {
public:
    YawController() = default;
    explicit YawController(const PIDController::Config& config);
    YawController(float kp, float ki, float kd);

    void set_config(const PIDController::Config& config);

    // Compute yaw rate command to reach target yaw
    // Angles are in radians, wrapped to [-PI, PI]
    float update(float dt, float target_yaw, float current_yaw);

    void reset();

    PIDController& controller() { return pid_; }

private:
    PIDController pid_;

    // Wrap angle to [-PI, PI]
    static float wrap_angle(float angle);
};

} // namespace control_algorithms
