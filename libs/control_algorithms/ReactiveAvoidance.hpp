#pragma once

#include "ProximityData.hpp"
#include <algorithm>
#include <cmath>

namespace control_algorithms {

// Reactive obstacle avoidance controller using repulsive potential fields
// Generates escape velocities based on proximity sensor data
class ReactiveAvoidance {
public:
    struct Config {
        float safety_distance = 2.0f;      // Start avoiding at this distance (meters)
        float critical_distance = 0.5f;    // Full stop threshold (meters)
        float max_avoidance_speed = 2.0f;  // Max escape velocity (m/s)
        float gain = 1.5f;                 // Repulsive force gain
    };

    // Avoidance state returned by update()
    enum class State {
        CLEAR,     // No obstacles within safety distance
        AVOIDING,  // Generating escape velocity
        CRITICAL   // Obstacle too close - emergency stop recommended
    };

    ReactiveAvoidance() = default;
    explicit ReactiveAvoidance(const Config& config);

    // Set configuration
    void set_config(const Config& config);
    const Config& config() const { return config_; }

    // Compute avoidance velocity from 6-direction proximity data
    // Returns the current avoidance state
    // out_vx/vy/vz: escape velocity in body frame (positive = away from obstacle)
    State update(const data_types::ProximityData& proximity,
                 float& out_vx, float& out_vy, float& out_vz);

    // Reset controller state
    void reset();

    // Get last computed state for telemetry
    State last_state() const { return last_state_; }
    float last_escape_magnitude() const { return last_escape_magnitude_; }

private:
    Config config_;
    State last_state_ = State::CLEAR;
    float last_escape_magnitude_ = 0.0f;

    // Compute repulsive force for a single direction
    // Returns force magnitude (always positive, pointing away from obstacle)
    float compute_repulsive_force(float distance) const;
};

} // namespace control_algorithms
