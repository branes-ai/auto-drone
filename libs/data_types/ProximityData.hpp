#pragma once

#include <cstdint>
#include <vector>
#include <algorithm>

namespace data_types {

// 6-direction proximity sensor data for obstacle avoidance
// Binary format: [front:4][back:4][left:4][right:4][up:4][down:4][timestamp:8] = 32 bytes
struct ProximityData {
    float distance_front = 999.0f;  // Distance to obstacle in front (meters)
    float distance_back = 999.0f;   // Distance to obstacle behind (meters)
    float distance_left = 999.0f;   // Distance to obstacle on left (meters)
    float distance_right = 999.0f;  // Distance to obstacle on right (meters)
    float distance_up = 999.0f;     // Distance to obstacle above (meters)
    float distance_down = 999.0f;   // Distance to obstacle below (meters)
    uint64_t timestamp_us = 0;      // Timestamp in microseconds since epoch

    // Default constructor
    ProximityData() = default;

    // Construct from parameters
    ProximityData(float front, float back, float left, float right,
                  float up, float down, uint64_t ts = 0);

    // Serialize to byte vector
    std::vector<uint8_t> serialize() const;

    // Deserialize from byte vector
    static ProximityData deserialize(const std::vector<uint8_t>& data);

    // Size in bytes when serialized
    static constexpr size_t SERIALIZED_SIZE = 32;

    // Factory for clear readings (no obstacles detected)
    static ProximityData clear(uint64_t ts = 0);

    // Helper: minimum distance across all directions
    float min_distance() const;

    // Helper: minimum horizontal distance (front/back/left/right)
    float min_horizontal_distance() const;

    // Helper: check if any obstacle is within threshold
    bool has_obstacle(float threshold = 2.0f) const;

    // Helper: check if any horizontal obstacle is within threshold
    bool has_horizontal_obstacle(float threshold = 2.0f) const;
};

} // namespace data_types
