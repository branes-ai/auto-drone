#pragma once

#include <cstdint>
#include <vector>
#include <cstring>

namespace data_types {

// Velocity command for drone control
// Binary format: [vx:4][vy:4][vz:4][yaw_rate:4][priority:1][source:1][padding:2][timestamp:8] = 28 bytes
struct VelocityCommand {
    float vx = 0.0f;           // X velocity (m/s, body frame forward)
    float vy = 0.0f;           // Y velocity (m/s, body frame right)
    float vz = 0.0f;           // Z velocity (m/s, body frame down)
    float yaw_rate = 0.0f;     // Yaw rate (rad/s, positive clockwise from above)
    uint64_t timestamp_us = 0; // Timestamp in microseconds

    // Priority for command arbitration (higher = more priority)
    enum Priority : uint8_t {
        LOW = 0,       // e.g., waypoint following
        NORMAL = 1,    // e.g., manual override
        HIGH = 2,      // e.g., reactive avoidance
        CRITICAL = 3   // e.g., emergency stop
    };
    uint8_t priority = LOW;

    // Source of the command for debugging/logging
    enum Source : uint8_t {
        UNKNOWN = 0,
        WAYPOINT_MANAGER = 1,
        OBSTACLE_AVOIDANCE = 2,
        MANUAL_CONTROL = 3,
        EMERGENCY = 4
    };
    uint8_t source = UNKNOWN;

    VelocityCommand() = default;
    VelocityCommand(float vx, float vy, float vz, float yaw_rate = 0.0f,
                    uint8_t priority = LOW, uint8_t source = UNKNOWN,
                    uint64_t timestamp = 0);

    // Serialize to byte vector
    std::vector<uint8_t> serialize() const;

    // Deserialize from byte vector
    static VelocityCommand deserialize(const std::vector<uint8_t>& data);

    // Size in bytes when serialized
    static constexpr size_t SERIALIZED_SIZE = 28;

    // Create a zero velocity (hover) command
    static VelocityCommand hover(uint8_t priority = LOW, uint8_t source = UNKNOWN);

    // Create an emergency stop command
    static VelocityCommand emergency_stop();
};

} // namespace data_types
