#pragma once

#include "VelocityCommand.hpp"
#include <map>
#include <cstdint>

namespace obstacle_avoidance {

// Command arbiter for selecting between multiple velocity command sources
// Uses priority-based selection with staleness filtering
class CommandArbiter {
public:
    struct Config {
        uint64_t command_timeout_us = 500000;  // 500ms staleness threshold
    };

    CommandArbiter() = default;
    explicit CommandArbiter(const Config& config);

    // Set configuration
    void set_config(const Config& config);
    const Config& config() const { return config_; }

    // Add or update a command from a source
    // Commands are keyed by their source field
    void add_command(const data_types::VelocityCommand& cmd);

    // Clear all stored commands
    void clear();

    // Select the highest priority non-stale command
    // Returns hover command if no valid commands exist
    // current_time_us: current timestamp in microseconds
    data_types::VelocityCommand arbitrate(uint64_t current_time_us) const;

    // Get the winning source from last arbitration (for logging)
    uint8_t last_winning_source() const { return last_winning_source_; }

    // Check if a specific source has a valid (non-stale) command
    bool has_valid_command(uint8_t source, uint64_t current_time_us) const;

    // Get count of stored commands (for debugging)
    size_t command_count() const { return commands_by_source_.size(); }

private:
    Config config_;
    std::map<uint8_t, data_types::VelocityCommand> commands_by_source_;
    mutable uint8_t last_winning_source_ = data_types::VelocityCommand::UNKNOWN;

    // Check if a command is stale
    bool is_stale(const data_types::VelocityCommand& cmd, uint64_t current_time_us) const;
};

} // namespace obstacle_avoidance
