#include "CommandArbiter.hpp"

namespace obstacle_avoidance {

CommandArbiter::CommandArbiter(const Config& config)
    : config_(config) {}

void CommandArbiter::set_config(const Config& config) {
    config_ = config;
}

void CommandArbiter::add_command(const data_types::VelocityCommand& cmd) {
    commands_by_source_[cmd.source] = cmd;
}

void CommandArbiter::clear() {
    commands_by_source_.clear();
    last_winning_source_ = data_types::VelocityCommand::UNKNOWN;
}

bool CommandArbiter::is_stale(const data_types::VelocityCommand& cmd, uint64_t current_time_us) const {
    // Handle timestamp wraparound (unlikely but safe)
    if (cmd.timestamp_us > current_time_us) {
        return false;  // Future timestamp, assume valid
    }
    return (current_time_us - cmd.timestamp_us) > config_.command_timeout_us;
}

bool CommandArbiter::has_valid_command(uint8_t source, uint64_t current_time_us) const {
    auto it = commands_by_source_.find(source);
    if (it == commands_by_source_.end()) {
        return false;
    }
    return !is_stale(it->second, current_time_us);
}

data_types::VelocityCommand CommandArbiter::arbitrate(uint64_t current_time_us) const {
    const data_types::VelocityCommand* best = nullptr;

    for (const auto& [source, cmd] : commands_by_source_) {
        // Skip stale commands
        if (is_stale(cmd, current_time_us)) {
            continue;
        }

        // Select if no current best
        if (best == nullptr) {
            best = &cmd;
            continue;
        }

        // Prefer higher priority
        if (cmd.priority > best->priority) {
            best = &cmd;
            continue;
        }

        // On priority tie, prefer most recent timestamp
        if (cmd.priority == best->priority && cmd.timestamp_us > best->timestamp_us) {
            best = &cmd;
        }
    }

    // If no valid command found, return hover
    if (best == nullptr) {
        last_winning_source_ = data_types::VelocityCommand::UNKNOWN;
        return data_types::VelocityCommand::hover(
            data_types::VelocityCommand::LOW,
            data_types::VelocityCommand::COMMAND_ARBITER);
    }

    // Return the winning command with source marked as COMMAND_ARBITER
    last_winning_source_ = best->source;

    data_types::VelocityCommand result = *best;
    result.source = data_types::VelocityCommand::COMMAND_ARBITER;
    result.timestamp_us = current_time_us;
    return result;
}

} // namespace obstacle_avoidance
