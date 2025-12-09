#include "TargetTracker.hpp"
#include <cmath>
#include <algorithm>

namespace control_algorithms {

TargetTracker::TargetTracker() {
    // Default gains for image-based tracking
    config_.image_x.gains = {1.0f, 0.0f, 0.1f};
    config_.image_y.gains = {1.0f, 0.0f, 0.1f};
    config_.distance.gains = {0.5f, 0.05f, 0.1f};

    pid_image_x_.set_config(config_.image_x);
    pid_image_y_.set_config(config_.image_y);
    pid_distance_.set_config(config_.distance);
}

TargetTracker::TargetTracker(const Config& config)
    : config_(config),
      pid_image_x_(config.image_x),
      pid_image_y_(config.image_y),
      pid_distance_(config.distance) {}

void TargetTracker::set_config(const Config& config) {
    config_ = config;
    pid_image_x_.set_config(config.image_x);
    pid_image_y_.set_config(config.image_y);
    pid_distance_.set_config(config.distance);
}

void TargetTracker::set_limits(const Limits& limits) {
    limits_ = limits;

    // Apply limits to controllers
    PIDController::Limits yaw_limits;
    yaw_limits.output_min = -limits.max_yaw_rate;
    yaw_limits.output_max = limits.max_yaw_rate;
    pid_image_x_.set_limits(yaw_limits);

    PIDController::Limits vert_limits;
    vert_limits.output_min = -limits.max_vertical_velocity;
    vert_limits.output_max = limits.max_vertical_velocity;
    pid_image_y_.set_limits(vert_limits);

    PIDController::Limits fwd_limits;
    fwd_limits.output_min = -limits.max_forward_velocity;
    fwd_limits.output_max = limits.max_forward_velocity;
    pid_distance_.set_limits(fwd_limits);
}

void TargetTracker::update(float dt,
                            float bbox_cx, float bbox_cy,
                            float target_depth,
                            float& out_vx, float& out_vy, float& out_vz, float& out_yaw_rate) {
    // Normalize pixel error to [-1, 1] range
    float image_cx = config_.image_width / 2.0f;
    float image_cy = config_.image_height / 2.0f;

    float error_x = (bbox_cx - image_cx) / image_cx;  // Positive = target is right of center
    float error_y = (bbox_cy - image_cy) / image_cy;  // Positive = target is below center

    // Horizontal error -> yaw rate (turn to center target)
    // Positive error (target right) -> positive yaw rate (turn right)
    out_yaw_rate = pid_image_x_.update(dt, 0.0f, -error_x);

    // Vertical error -> vertical velocity
    // Positive error (target below) -> positive vz (descend to center target)
    out_vz = pid_image_y_.update(dt, 0.0f, -error_y);

    // Distance error -> forward velocity
    // Positive error (target far) -> positive vx (move forward)
    out_vx = pid_distance_.update(dt, config_.target_distance, target_depth);

    // No lateral movement in basic 2D tracking
    out_vy = 0.0f;

    // Apply limits
    out_vx = std::clamp(out_vx, -limits_.max_forward_velocity, limits_.max_forward_velocity);
    out_vy = std::clamp(out_vy, -limits_.max_lateral_velocity, limits_.max_lateral_velocity);
    out_vz = std::clamp(out_vz, -limits_.max_vertical_velocity, limits_.max_vertical_velocity);
    out_yaw_rate = std::clamp(out_yaw_rate, -limits_.max_yaw_rate, limits_.max_yaw_rate);
}

void TargetTracker::update_3d(float dt,
                               float target_x, float target_y, float target_z,
                               float& out_vx, float& out_vy, float& out_vz, float& out_yaw_rate) {
    // Compute distance and bearing to target
    float distance = std::sqrt(target_x * target_x + target_y * target_y);
    float bearing = std::atan2(target_y, target_x);  // Angle from forward axis

    // Distance control -> forward velocity
    out_vx = pid_distance_.update(dt, config_.target_distance, distance);

    // Bearing control -> yaw rate (turn to face target)
    // Positive bearing (target to the right) -> positive yaw rate
    out_yaw_rate = pid_image_x_.update(dt, 0.0f, -bearing);

    // Vertical offset -> vertical velocity
    // Positive target_z (target below) -> positive vz (descend)
    out_vz = pid_image_y_.update(dt, 0.0f, -target_z);

    // Lateral velocity for strafing (optional, based on lateral offset)
    out_vy = 0.0f;  // Could use target_y for strafing if yaw rate is limited

    // Apply limits
    out_vx = std::clamp(out_vx, -limits_.max_forward_velocity, limits_.max_forward_velocity);
    out_vy = std::clamp(out_vy, -limits_.max_lateral_velocity, limits_.max_lateral_velocity);
    out_vz = std::clamp(out_vz, -limits_.max_vertical_velocity, limits_.max_vertical_velocity);
    out_yaw_rate = std::clamp(out_yaw_rate, -limits_.max_yaw_rate, limits_.max_yaw_rate);
}

void TargetTracker::reset() {
    pid_image_x_.reset();
    pid_image_y_.reset();
    pid_distance_.reset();
}

} // namespace control_algorithms
