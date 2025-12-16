// CommandArbiterNode.cpp
// Subscribes to nominal and reactive velocity commands, arbitrates based on
// priority, and publishes the winning command to the final velocity topic.

#include "ZenohInterface.hpp"
#include "VelocityCommand.hpp"
#include "CommandArbiter.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <mutex>
#include <string>

namespace {

std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
}

// State protected by mutex
std::mutex g_state_mutex;
obstacle_avoidance::CommandArbiter g_arbiter;

// Configuration
struct Config {
    float arbiter_rate = 50.0f;         // Hz
    float command_timeout = 0.5f;       // seconds
    std::string connect_endpoint;       // Remote Zenoh endpoint
};

void print_usage() {
    std::cout << "Usage: command_arbiter_node [options]\n"
              << "Options:\n"
              << "  --rate <val>      Arbiter rate Hz (default: 50)\n"
              << "  --timeout <val>   Command timeout seconds (default: 0.5)\n"
              << "  --connect <ep>    Remote Zenoh endpoint (e.g., tcp/192.168.1.10:7447)\n"
              << std::endl;
}

void parse_args(int argc, char* argv[], Config& config) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage();
            std::exit(0);
        }
        if (arg == "--rate" && i + 1 < argc) config.arbiter_rate = std::stof(argv[++i]);
        else if (arg == "--timeout" && i + 1 < argc) config.command_timeout = std::stof(argv[++i]);
        else if (arg == "--connect" && i + 1 < argc) config.connect_endpoint = argv[++i];
    }
}

const char* source_name(uint8_t source) {
    switch (source) {
        case data_types::VelocityCommand::UNKNOWN: return "UNKNOWN";
        case data_types::VelocityCommand::WAYPOINT_MANAGER: return "WAYPOINT";
        case data_types::VelocityCommand::OBSTACLE_AVOIDANCE: return "AVOIDANCE";
        case data_types::VelocityCommand::MANUAL_CONTROL: return "MANUAL";
        case data_types::VelocityCommand::EMERGENCY: return "EMERGENCY";
        case data_types::VelocityCommand::COMMAND_ARBITER: return "ARBITER";
        default: return "OTHER";
    }
}

const char* priority_name(uint8_t priority) {
    switch (priority) {
        case data_types::VelocityCommand::LOW: return "LOW";
        case data_types::VelocityCommand::NORMAL: return "NORMAL";
        case data_types::VelocityCommand::HIGH: return "HIGH";
        case data_types::VelocityCommand::CRITICAL: return "CRITICAL";
        default: return "UNKNOWN";
    }
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    Config config;
    parse_args(argc, argv, config);

    std::cout << "=== Command Arbiter Node ===" << std::endl;
    std::cout << "Arbiter rate: " << config.arbiter_rate << " Hz" << std::endl;
    std::cout << "Command timeout: " << config.command_timeout << " s" << std::endl;
    std::cout << std::endl;

    // Initialize Zenoh session
    std::unique_ptr<zenoh_interface::Session> session_ptr;
    if (!config.connect_endpoint.empty()) {
        std::cout << "Connecting to Zenoh at " << config.connect_endpoint << std::endl;
        auto session_config = zenoh_interface::SessionConfig::connect_to(config.connect_endpoint);
        session_ptr = std::make_unique<zenoh_interface::Session>(session_config);
    } else {
        std::cout << "Using local Zenoh scouting" << std::endl;
        session_ptr = std::make_unique<zenoh_interface::Session>();
    }
    auto& session = *session_ptr;
    if (!session.is_valid()) {
        std::cerr << "Failed to create Zenoh session. Exiting." << std::endl;
        return 1;
    }

    // Configure arbiter
    {
        obstacle_avoidance::CommandArbiter::Config arbiter_config;
        arbiter_config.command_timeout_us = static_cast<uint64_t>(config.command_timeout * 1000000.0f);
        std::lock_guard<std::mutex> lock(g_state_mutex);
        g_arbiter.set_config(arbiter_config);
    }

    // Zenoh key expressions
    const std::string nominal_vel_key = "robot/drone/cmd/nominal_vel";
    const std::string reactive_vel_key = "robot/drone/cmd/reactive_vel";
    const std::string velocity_key = "robot/drone/cmd/velocity";

    // Subscribe to nominal velocity commands (from WaypointManager)
    session.subscribe(nominal_vel_key, [](const std::string&, const std::vector<uint8_t>& payload) {
        try {
            auto cmd = data_types::VelocityCommand::deserialize(payload);
            std::lock_guard<std::mutex> lock(g_state_mutex);
            g_arbiter.add_command(cmd);
        } catch (const std::exception& e) {
            std::cerr << "[Nominal] Deserialization error: " << e.what() << std::endl;
        }
    });

    // Subscribe to reactive velocity commands (from ObstacleAvoidanceNode)
    session.subscribe(reactive_vel_key, [](const std::string&, const std::vector<uint8_t>& payload) {
        try {
            auto cmd = data_types::VelocityCommand::deserialize(payload);
            std::lock_guard<std::mutex> lock(g_state_mutex);
            g_arbiter.add_command(cmd);
        } catch (const std::exception& e) {
            std::cerr << "[Reactive] Deserialization error: " << e.what() << std::endl;
        }
    });

    std::cout << "Subscribed to:" << std::endl;
    std::cout << "  Nominal: " << nominal_vel_key << std::endl;
    std::cout << "  Reactive: " << reactive_vel_key << std::endl;
    std::cout << "Publishing to:" << std::endl;
    std::cout << "  Velocity: " << velocity_key << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;
    std::cout << std::endl;

    // Timing
    const auto arbiter_interval = std::chrono::microseconds(
        static_cast<int>(1000000.0f / config.arbiter_rate));
    auto last_arbiter_time = std::chrono::steady_clock::now();

    int cmd_count = 0;
    uint8_t last_winning_source = data_types::VelocityCommand::UNKNOWN;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();

        if (now - last_arbiter_time >= arbiter_interval) {
            last_arbiter_time = now;

            auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            // Arbitrate
            data_types::VelocityCommand cmd;
            uint8_t winning_source;
            {
                std::lock_guard<std::mutex> lock(g_state_mutex);
                cmd = g_arbiter.arbitrate(now_us);
                winning_source = g_arbiter.last_winning_source();
            }

            // Log source changes
            if (winning_source != last_winning_source) {
                std::cout << "[Source] " << source_name(last_winning_source)
                          << " -> " << source_name(winning_source)
                          << " (priority=" << priority_name(cmd.priority) << ")" << std::endl;
                last_winning_source = winning_source;
            }

            // Publish arbitrated command
            if (session.publish(velocity_key, cmd.serialize())) {
                cmd_count++;
                if (cmd_count % static_cast<int>(config.arbiter_rate * 2) == 0) {
                    std::cout << "[Arbiter] source=" << source_name(winning_source)
                              << " vel=(" << cmd.vx << ", " << cmd.vy << ", " << cmd.vz << ")"
                              << " priority=" << priority_name(cmd.priority) << std::endl;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    std::cout << std::endl;
    std::cout << "Shutting down..." << std::endl;
    std::cout << "Total commands published: " << cmd_count << std::endl;

    return 0;
}
