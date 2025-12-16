#include "ReactiveAvoidance.hpp"

namespace control_algorithms {

ReactiveAvoidance::ReactiveAvoidance(const Config& config)
    : config_(config) {}

void ReactiveAvoidance::set_config(const Config& config) {
    config_ = config;
}

void ReactiveAvoidance::reset() {
    last_state_ = State::CLEAR;
    last_escape_magnitude_ = 0.0f;
}

float ReactiveAvoidance::compute_repulsive_force(float distance) const {
    // No force if beyond safety distance
    if (distance >= config_.safety_distance) {
        return 0.0f;
    }

    // Clamp distance to avoid division by zero
    float d = std::max(distance, 0.01f);

    // Repulsive potential field: force = gain * (1/d - 1/safety_distance)
    // This gives zero force at safety_distance and increasing force as d decreases
    float force = config_.gain * (1.0f / d - 1.0f / config_.safety_distance);

    return std::max(0.0f, force);
}

ReactiveAvoidance::State ReactiveAvoidance::update(
    const data_types::ProximityData& proximity,
    float& out_vx, float& out_vy, float& out_vz)
{
    // Initialize outputs
    out_vx = 0.0f;
    out_vy = 0.0f;
    out_vz = 0.0f;

    // Check for critical distance (emergency stop)
    float min_dist = proximity.min_distance();
    if (min_dist < config_.critical_distance) {
        last_state_ = State::CRITICAL;
        last_escape_magnitude_ = 0.0f;
        return State::CRITICAL;
    }

    // Check if any obstacle is within safety distance
    if (!proximity.has_obstacle(config_.safety_distance)) {
        last_state_ = State::CLEAR;
        last_escape_magnitude_ = 0.0f;
        return State::CLEAR;
    }

    // Compute repulsive velocities for each direction
    // Positive velocity = move away from obstacle

    // Front obstacle: move backward (negative vx in body frame)
    float force_front = compute_repulsive_force(proximity.distance_front);
    out_vx -= force_front;

    // Back obstacle: move forward (positive vx in body frame)
    float force_back = compute_repulsive_force(proximity.distance_back);
    out_vx += force_back;

    // Left obstacle: move right (positive vy in body frame, right is +Y)
    float force_left = compute_repulsive_force(proximity.distance_left);
    out_vy += force_left;

    // Right obstacle: move left (negative vy in body frame)
    float force_right = compute_repulsive_force(proximity.distance_right);
    out_vy -= force_right;

    // Up obstacle: move down (positive vz in body frame, down is +Z)
    float force_up = compute_repulsive_force(proximity.distance_up);
    out_vz += force_up;

    // Down obstacle: move up (negative vz in body frame)
    float force_down = compute_repulsive_force(proximity.distance_down);
    out_vz -= force_down;

    // Compute escape velocity magnitude
    float magnitude = std::sqrt(out_vx * out_vx + out_vy * out_vy + out_vz * out_vz);

    // Clamp to max avoidance speed while preserving direction
    if (magnitude > config_.max_avoidance_speed) {
        float scale = config_.max_avoidance_speed / magnitude;
        out_vx *= scale;
        out_vy *= scale;
        out_vz *= scale;
        magnitude = config_.max_avoidance_speed;
    }

    last_state_ = State::AVOIDING;
    last_escape_magnitude_ = magnitude;
    return State::AVOIDING;
}

} // namespace control_algorithms
