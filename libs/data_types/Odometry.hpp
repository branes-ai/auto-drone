#pragma once

#include <cstdint>
#include <vector>

namespace data_types {

// Odometry data container with serialization support
// Binary format: [x:4][y:4][z:4][roll:4][pitch:4][yaw:4][timestamp:8] = 32 bytes
struct Odometry {
    float x = 0.0f;      // Position X (meters)
    float y = 0.0f;      // Position Y (meters)
    float z = 0.0f;      // Position Z (meters)
    float roll = 0.0f;   // Rotation around X-axis (radians)
    float pitch = 0.0f;  // Rotation around Y-axis (radians)
    float yaw = 0.0f;    // Rotation around Z-axis (radians)
    uint64_t timestamp_us = 0;  // Timestamp in microseconds since epoch

    // Construct from parameters
    Odometry() = default;
    Odometry(float x, float y, float z, float roll, float pitch, float yaw, uint64_t ts = 0);

    // Serialize to byte vector
    std::vector<uint8_t> serialize() const;

    // Deserialize from byte vector
    static Odometry deserialize(const std::vector<uint8_t>& data);

    // Size in bytes when serialized
    static constexpr size_t SERIALIZED_SIZE = 32;
};

} // namespace data_types
