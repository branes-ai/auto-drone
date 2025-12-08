#pragma once

#include <cstdint>
#include <vector>
#include <cstring>

namespace data_types {

// Single waypoint with position and optional yaw
// Binary format: [x:4][y:4][z:4][yaw:4][speed:4][id:4][flags:1][padding:3] = 28 bytes
struct Waypoint {
    float x = 0.0f;           // Position X (meters, NED frame)
    float y = 0.0f;           // Position Y (meters, NED frame)
    float z = 0.0f;           // Position Z (meters, NED frame - negative is up)
    float yaw = 0.0f;         // Desired yaw at waypoint (radians)
    float speed = 1.0f;       // Desired approach speed (m/s)
    uint32_t id = 0;          // Waypoint identifier

    // Flags for waypoint behavior
    enum Flags : uint8_t {
        NONE = 0,
        HOLD_YAW = 1 << 0,    // Maintain specified yaw during approach
        HOVER = 1 << 1,       // Hover at waypoint briefly
        FINAL = 1 << 2        // Last waypoint in sequence
    };
    uint8_t flags = NONE;

    Waypoint() = default;
    Waypoint(float x, float y, float z, float yaw = 0.0f, float speed = 1.0f,
             uint32_t id = 0, uint8_t flags = NONE);

    // Serialize to byte vector
    std::vector<uint8_t> serialize() const;

    // Deserialize from byte vector
    static Waypoint deserialize(const std::vector<uint8_t>& data);

    // Size in bytes when serialized
    static constexpr size_t SERIALIZED_SIZE = 28;
};

// Waypoint list message for publishing sequences
// Binary format: [count:4][waypoint:28]...
struct WaypointList {
    std::vector<Waypoint> waypoints;
    uint64_t timestamp_us = 0;

    WaypointList() = default;

    void add(const Waypoint& wp) { waypoints.push_back(wp); }
    void clear() { waypoints.clear(); }
    size_t size() const { return waypoints.size(); }
    bool empty() const { return waypoints.empty(); }

    const Waypoint& operator[](size_t idx) const { return waypoints[idx]; }
    Waypoint& operator[](size_t idx) { return waypoints[idx]; }

    // Serialize to byte vector
    std::vector<uint8_t> serialize() const;

    // Deserialize from byte vector
    static WaypointList deserialize(const std::vector<uint8_t>& data);
};

} // namespace data_types
