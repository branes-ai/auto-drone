#pragma once

#include "PIDController.hpp"

namespace control_algorithms {

// Target tracking controller for following detected objects
// Computes velocity commands to keep the target centered in the camera frame
// and at a desired distance from the drone.
class TargetTracker {
public:
    struct Config {
        // Image center tracking (pixel error -> yaw/pitch velocity)
        PIDController::Config image_x;  // Horizontal centering -> yaw rate
        PIDController::Config image_y;  // Vertical centering -> pitch/altitude

        // Distance tracking (depth error -> forward velocity)
        PIDController::Config distance;

        // Image dimensions for normalization
        int image_width = 640;
        int image_height = 480;

        // Target setpoints
        float target_distance = 3.0f;  // Desired distance to target (meters)
    };

    struct Limits {
        float max_forward_velocity = 2.0f;   // m/s
        float max_lateral_velocity = 1.0f;   // m/s
        float max_vertical_velocity = 1.0f;  // m/s
        float max_yaw_rate = 1.0f;           // rad/s
    };

    TargetTracker();
    explicit TargetTracker(const Config& config);

    void set_config(const Config& config);
    const Config& config() const { return config_; }

    void set_limits(const Limits& limits);
    const Limits& limits() const { return limits_; }

    // Update controller with target position
    // Returns velocity command (vx, vy, vz, yaw_rate)
    // bbox_cx, bbox_cy: target center in pixels
    // target_depth: estimated distance to target in meters
    void update(float dt,
                float bbox_cx, float bbox_cy,
                float target_depth,
                float& out_vx, float& out_vy, float& out_vz, float& out_yaw_rate);

    // Update controller with 3D target position (in drone body frame)
    // target_x: forward (positive = in front)
    // target_y: right (positive = to the right)
    // target_z: down (positive = below)
    void update_3d(float dt,
                   float target_x, float target_y, float target_z,
                   float& out_vx, float& out_vy, float& out_vz, float& out_yaw_rate);

    // Reset controller state
    void reset();

    // Access individual controllers for tuning
    PIDController& image_x_controller() { return pid_image_x_; }
    PIDController& image_y_controller() { return pid_image_y_; }
    PIDController& distance_controller() { return pid_distance_; }

private:
    Config config_;
    Limits limits_;

    PIDController pid_image_x_;   // Horizontal image error -> yaw
    PIDController pid_image_y_;   // Vertical image error -> vertical velocity
    PIDController pid_distance_;  // Distance error -> forward velocity
};

} // namespace control_algorithms
