#include "Waypoint.hpp"
#include <stdexcept>
#include <cstring>

namespace data_types {

// --- Waypoint ---

Waypoint::Waypoint(float x, float y, float z, float yaw, float speed,
                   uint32_t id, uint8_t flags)
    : x(x), y(y), z(z), yaw(yaw), speed(speed), id(id), flags(flags) {}

std::vector<uint8_t> Waypoint::serialize() const {
    std::vector<uint8_t> data(SERIALIZED_SIZE, 0);
    size_t offset = 0;

    std::memcpy(data.data() + offset, &x, sizeof(x)); offset += sizeof(x);
    std::memcpy(data.data() + offset, &y, sizeof(y)); offset += sizeof(y);
    std::memcpy(data.data() + offset, &z, sizeof(z)); offset += sizeof(z);
    std::memcpy(data.data() + offset, &yaw, sizeof(yaw)); offset += sizeof(yaw);
    std::memcpy(data.data() + offset, &speed, sizeof(speed)); offset += sizeof(speed);
    std::memcpy(data.data() + offset, &id, sizeof(id)); offset += sizeof(id);
    std::memcpy(data.data() + offset, &flags, sizeof(flags));
    // 3 bytes padding at the end for alignment

    return data;
}

Waypoint Waypoint::deserialize(const std::vector<uint8_t>& data) {
    if (data.size() < SERIALIZED_SIZE) {
        throw std::runtime_error("Waypoint: insufficient data for deserialization");
    }

    Waypoint wp;
    size_t offset = 0;

    std::memcpy(&wp.x, data.data() + offset, sizeof(wp.x)); offset += sizeof(wp.x);
    std::memcpy(&wp.y, data.data() + offset, sizeof(wp.y)); offset += sizeof(wp.y);
    std::memcpy(&wp.z, data.data() + offset, sizeof(wp.z)); offset += sizeof(wp.z);
    std::memcpy(&wp.yaw, data.data() + offset, sizeof(wp.yaw)); offset += sizeof(wp.yaw);
    std::memcpy(&wp.speed, data.data() + offset, sizeof(wp.speed)); offset += sizeof(wp.speed);
    std::memcpy(&wp.id, data.data() + offset, sizeof(wp.id)); offset += sizeof(wp.id);
    std::memcpy(&wp.flags, data.data() + offset, sizeof(wp.flags));

    return wp;
}

// --- WaypointList ---

std::vector<uint8_t> WaypointList::serialize() const {
    // Header: count (4 bytes) + timestamp (8 bytes)
    size_t header_size = sizeof(uint32_t) + sizeof(uint64_t);
    size_t total_size = header_size + waypoints.size() * Waypoint::SERIALIZED_SIZE;
    std::vector<uint8_t> data(total_size);

    size_t offset = 0;
    uint32_t count = static_cast<uint32_t>(waypoints.size());
    std::memcpy(data.data() + offset, &count, sizeof(count)); offset += sizeof(count);
    std::memcpy(data.data() + offset, &timestamp_us, sizeof(timestamp_us)); offset += sizeof(timestamp_us);

    for (const auto& wp : waypoints) {
        auto wp_data = wp.serialize();
        std::memcpy(data.data() + offset, wp_data.data(), wp_data.size());
        offset += wp_data.size();
    }

    return data;
}

WaypointList WaypointList::deserialize(const std::vector<uint8_t>& data) {
    size_t header_size = sizeof(uint32_t) + sizeof(uint64_t);
    if (data.size() < header_size) {
        throw std::runtime_error("WaypointList: insufficient data for header");
    }

    WaypointList list;
    size_t offset = 0;

    uint32_t count;
    std::memcpy(&count, data.data() + offset, sizeof(count)); offset += sizeof(count);
    std::memcpy(&list.timestamp_us, data.data() + offset, sizeof(list.timestamp_us)); offset += sizeof(list.timestamp_us);

    size_t expected_size = header_size + count * Waypoint::SERIALIZED_SIZE;
    if (data.size() < expected_size) {
        throw std::runtime_error("WaypointList: insufficient data for waypoints");
    }

    list.waypoints.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        std::vector<uint8_t> wp_data(data.begin() + offset,
                                      data.begin() + offset + Waypoint::SERIALIZED_SIZE);
        list.waypoints.push_back(Waypoint::deserialize(wp_data));
        offset += Waypoint::SERIALIZED_SIZE;
    }

    return list;
}

} // namespace data_types
