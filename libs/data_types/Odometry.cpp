#include "Odometry.hpp"
#include <cstring>
#include <stdexcept>

namespace data_types {

Odometry::Odometry(float x_, float y_, float z_, float roll_, float pitch_, float yaw_, uint64_t ts)
    : x(x_), y(y_), z(z_), roll(roll_), pitch(pitch_), yaw(yaw_), timestamp_us(ts) {}

std::vector<uint8_t> Odometry::serialize() const {
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

    write_float(0, x);
    write_float(4, y);
    write_float(8, z);
    write_float(12, roll);
    write_float(16, pitch);
    write_float(20, yaw);
    write_u64(24, timestamp_us);

    return result;
}

Odometry Odometry::deserialize(const std::vector<uint8_t>& data) {
    if (data.size() < SERIALIZED_SIZE) {
        throw std::runtime_error("Odometry::deserialize: data too small");
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

    Odometry odom;
    odom.x = read_float(0);
    odom.y = read_float(4);
    odom.z = read_float(8);
    odom.roll = read_float(12);
    odom.pitch = read_float(16);
    odom.yaw = read_float(20);
    odom.timestamp_us = read_u64(24);

    return odom;
}

} // namespace data_types
