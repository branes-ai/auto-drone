#include "ProximityData.hpp"
#include <cstring>
#include <stdexcept>

namespace data_types {

ProximityData::ProximityData(float front, float back, float left, float right,
                             float up, float down, uint64_t ts)
    : distance_front(front), distance_back(back), distance_left(left),
      distance_right(right), distance_up(up), distance_down(down),
      timestamp_us(ts) {}

std::vector<uint8_t> ProximityData::serialize() const {
    std::vector<uint8_t> result(SERIALIZED_SIZE);

    // Helper to write float as little-endian bytes
    auto write_float = [&result](size_t offset, float value) {
        uint32_t bits;
        std::memcpy(&bits, &value, sizeof(float));
        result[offset] = static_cast<uint8_t>(bits & 0xFF);
        result[offset + 1] = static_cast<uint8_t>((bits >> 8) & 0xFF);
        result[offset + 2] = static_cast<uint8_t>((bits >> 16) & 0xFF);
        result[offset + 3] = static_cast<uint8_t>((bits >> 24) & 0xFF);
    };

    // Helper to write uint64 as little-endian bytes
    auto write_u64 = [&result](size_t offset, uint64_t value) {
        for (int i = 0; i < 8; ++i) {
            result[offset + i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
        }
    };

    write_float(0, distance_front);
    write_float(4, distance_back);
    write_float(8, distance_left);
    write_float(12, distance_right);
    write_float(16, distance_up);
    write_float(20, distance_down);
    write_u64(24, timestamp_us);

    return result;
}

ProximityData ProximityData::deserialize(const std::vector<uint8_t>& data) {
    if (data.size() < SERIALIZED_SIZE) {
        throw std::runtime_error("ProximityData::deserialize: data too small");
    }

    // Helper to read float from little-endian bytes
    auto read_float = [&data](size_t offset) -> float {
        uint32_t bits = static_cast<uint32_t>(data[offset]) |
                        (static_cast<uint32_t>(data[offset + 1]) << 8) |
                        (static_cast<uint32_t>(data[offset + 2]) << 16) |
                        (static_cast<uint32_t>(data[offset + 3]) << 24);
        float value;
        std::memcpy(&value, &bits, sizeof(float));
        return value;
    };

    // Helper to read uint64 from little-endian bytes
    auto read_u64 = [&data](size_t offset) -> uint64_t {
        uint64_t value = 0;
        for (int i = 0; i < 8; ++i) {
            value |= static_cast<uint64_t>(data[offset + i]) << (i * 8);
        }
        return value;
    };

    ProximityData prox;
    prox.distance_front = read_float(0);
    prox.distance_back = read_float(4);
    prox.distance_left = read_float(8);
    prox.distance_right = read_float(12);
    prox.distance_up = read_float(16);
    prox.distance_down = read_float(20);
    prox.timestamp_us = read_u64(24);

    return prox;
}

ProximityData ProximityData::clear(uint64_t ts) {
    return ProximityData(999.0f, 999.0f, 999.0f, 999.0f, 999.0f, 999.0f, ts);
}

float ProximityData::min_distance() const {
    return std::min({distance_front, distance_back, distance_left,
                     distance_right, distance_up, distance_down});
}

float ProximityData::min_horizontal_distance() const {
    return std::min({distance_front, distance_back, distance_left, distance_right});
}

bool ProximityData::has_obstacle(float threshold) const {
    return min_distance() < threshold;
}

bool ProximityData::has_horizontal_obstacle(float threshold) const {
    return min_horizontal_distance() < threshold;
}

} // namespace data_types
