#include "VelocityCommand.hpp"
#include <stdexcept>
#include <chrono>

namespace data_types {

VelocityCommand::VelocityCommand(float vx, float vy, float vz, float yaw_rate,
                                 uint8_t priority, uint8_t source,
                                 uint64_t timestamp)
    : vx(vx), vy(vy), vz(vz), yaw_rate(yaw_rate),
      timestamp_us(timestamp), priority(priority), source(source) {}

std::vector<uint8_t> VelocityCommand::serialize() const {
    std::vector<uint8_t> data(SERIALIZED_SIZE, 0);
    size_t offset = 0;

    std::memcpy(data.data() + offset, &vx, sizeof(vx)); offset += sizeof(vx);
    std::memcpy(data.data() + offset, &vy, sizeof(vy)); offset += sizeof(vy);
    std::memcpy(data.data() + offset, &vz, sizeof(vz)); offset += sizeof(vz);
    std::memcpy(data.data() + offset, &yaw_rate, sizeof(yaw_rate)); offset += sizeof(yaw_rate);
    std::memcpy(data.data() + offset, &priority, sizeof(priority)); offset += sizeof(priority);
    std::memcpy(data.data() + offset, &source, sizeof(source)); offset += sizeof(source);
    // 2 bytes padding for alignment
    offset += 2;
    std::memcpy(data.data() + offset, &timestamp_us, sizeof(timestamp_us));

    return data;
}

VelocityCommand VelocityCommand::deserialize(const std::vector<uint8_t>& data) {
    if (data.size() < SERIALIZED_SIZE) {
        throw std::runtime_error("VelocityCommand: insufficient data for deserialization");
    }

    VelocityCommand cmd;
    size_t offset = 0;

    std::memcpy(&cmd.vx, data.data() + offset, sizeof(cmd.vx)); offset += sizeof(cmd.vx);
    std::memcpy(&cmd.vy, data.data() + offset, sizeof(cmd.vy)); offset += sizeof(cmd.vy);
    std::memcpy(&cmd.vz, data.data() + offset, sizeof(cmd.vz)); offset += sizeof(cmd.vz);
    std::memcpy(&cmd.yaw_rate, data.data() + offset, sizeof(cmd.yaw_rate)); offset += sizeof(cmd.yaw_rate);
    std::memcpy(&cmd.priority, data.data() + offset, sizeof(cmd.priority)); offset += sizeof(cmd.priority);
    std::memcpy(&cmd.source, data.data() + offset, sizeof(cmd.source)); offset += sizeof(cmd.source);
    offset += 2; // padding
    std::memcpy(&cmd.timestamp_us, data.data() + offset, sizeof(cmd.timestamp_us));

    return cmd;
}

VelocityCommand VelocityCommand::hover(uint8_t priority, uint8_t source) {
    auto now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return VelocityCommand(0.0f, 0.0f, 0.0f, 0.0f, priority, source, now);
}

VelocityCommand VelocityCommand::emergency_stop() {
    auto now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return VelocityCommand(0.0f, 0.0f, 0.0f, 0.0f, CRITICAL, EMERGENCY, now);
}

} // namespace data_types
