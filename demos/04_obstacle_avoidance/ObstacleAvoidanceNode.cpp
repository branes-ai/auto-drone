// ObstacleAvoidanceNode.cpp
// Subscribes to proximity sensor data and publishes reactive velocity commands
// when obstacles are detected within the safety distance.

#include "ZenohInterface.hpp"
#include "ProximityData.hpp"
#include "VelocityCommand.hpp"
#include "ReactiveAvoidance.hpp"

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
data_types::ProximityData g_proximity;
bool g_proximity_received = false;

// Configuration
struct Config {
    float safety_distance = 2.0f;       // Start avoiding at this distance (m)
    float critical_distance = 0.5f;     // Emergency stop threshold (m)
    float max_avoidance_speed = 2.0f;   // Max escape velocity (m/s)
    float gain = 1.5f;                  // Repulsive force gain
    float control_rate = 50.0f;         // Hz
    std::string connect_endpoint;       // Remote Zenoh endpoint
};

void print_usage() {
    std::cout << "Usage: obstacle_avoidance_node [options]\n"
              << "Options:\n"
              << "  --safety-dist <val>   Safety distance in meters (default: 2.0)\n"
              << "  --critical-dist <val> Critical distance in meters (default: 0.5)\n"
              << "  --max-vel <val>       Max avoidance velocity m/s (default: 2.0)\n"
              << "  --gain <val>          Repulsive force gain (default: 1.5)\n"
              << "  --rate <val>          Control rate Hz (default: 50)\n"
              << "  --connect <ep>        Remote Zenoh endpoint (e.g., tcp/192.168.1.10:7447)\n"
              << std::endl;
}

void parse_args(int argc, char* argv[], Config& config) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage();
            std::exit(0);
        }
        if (arg == "--safety-dist" && i + 1 < argc) config.safety_distance = std::stof(argv[++i]);
        else if (arg == "--critical-dist" && i + 1 < argc) config.critical_distance = std::stof(argv[++i]);
        else if (arg == "--max-vel" && i + 1 < argc) config.max_avoidance_speed = std::stof(argv[++i]);
        else if (arg == "--gain" && i + 1 < argc) config.gain = std::stof(argv[++i]);
        else if (arg == "--rate" && i + 1 < argc) config.control_rate = std::stof(argv[++i]);
        else if (arg == "--connect" && i + 1 < argc) config.connect_endpoint = argv[++i];
    }
}

const char* state_name(control_algorithms::ReactiveAvoidance::State state) {
    switch (state) {
        case control_algorithms::ReactiveAvoidance::State::CLEAR: return "CLEAR";
        case control_algorithms::ReactiveAvoidance::State::AVOIDING: return "AVOIDING";
        case control_algorithms::ReactiveAvoidance::State::CRITICAL: return "CRITICAL";
    }
    return "UNKNOWN";
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    Config config;
    parse_args(argc, argv, config);

    std::cout << "=== Obstacle Avoidance Node ===" << std::endl;
    std::cout << "Safety distance: " << config.safety_distance << " m" << std::endl;
    std::cout << "Critical distance: " << config.critical_distance << " m" << std::endl;
    std::cout << "Max avoidance velocity: " << config.max_avoidance_speed << " m/s" << std::endl;
    std::cout << "Gain: " << config.gain << std::endl;
    std::cout << "Control rate: " << config.control_rate << " Hz" << std::endl;
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

    // Zenoh key expressions
    const std::string proximity_key = "robot/drone/sensor/proximity";
    const std::string reactive_vel_key = "robot/drone/cmd/reactive_vel";

    // Subscribe to proximity data
    session.subscribe(proximity_key, [](const std::string&, const std::vector<uint8_t>& payload) {
        try {
            auto prox = data_types::ProximityData::deserialize(payload);
            std::lock_guard<std::mutex> lock(g_state_mutex);
            g_proximity = prox;
            g_proximity_received = true;
        } catch (const std::exception& e) {
            std::cerr << "[Proximity] Deserialization error: " << e.what() << std::endl;
        }
    });

    std::cout << "Subscribed to: " << proximity_key << std::endl;
    std::cout << "Publishing to: " << reactive_vel_key << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;
    std::cout << std::endl;

    // Setup reactive avoidance controller
    control_algorithms::ReactiveAvoidance::Config avoidance_config;
    avoidance_config.safety_distance = config.safety_distance;
    avoidance_config.critical_distance = config.critical_distance;
    avoidance_config.max_avoidance_speed = config.max_avoidance_speed;
    avoidance_config.gain = config.gain;
    control_algorithms::ReactiveAvoidance avoidance(avoidance_config);

    // Timing
    const auto control_interval = std::chrono::microseconds(
        static_cast<int>(1000000.0f / config.control_rate));
    auto last_control_time = std::chrono::steady_clock::now();

    int cmd_count = 0;
    auto last_state = control_algorithms::ReactiveAvoidance::State::CLEAR;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();

        if (now - last_control_time >= control_interval) {
            last_control_time = now;

            // Get current proximity data
            data_types::ProximityData prox;
            bool have_data = false;
            {
                std::lock_guard<std::mutex> lock(g_state_mutex);
                if (g_proximity_received) {
                    prox = g_proximity;
                    have_data = true;
                }
            }

            if (!have_data) {
                continue;
            }

            // Compute avoidance velocity
            float vx, vy, vz;
            auto state = avoidance.update(prox, vx, vy, vz);

            // Log state changes
            if (state != last_state) {
                std::cout << "[State] " << state_name(last_state) << " -> " << state_name(state) << std::endl;
                last_state = state;
            }

            // Only publish if avoiding or critical
            if (state == control_algorithms::ReactiveAvoidance::State::CLEAR) {
                continue;  // No obstacle, don't spam reactive commands
            }

            // Create velocity command
            auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            data_types::VelocityCommand cmd;
            if (state == control_algorithms::ReactiveAvoidance::State::CRITICAL) {
                // Emergency stop
                cmd = data_types::VelocityCommand::emergency_stop();
                cmd.timestamp_us = now_us;
            } else {
                // Avoidance velocity
                cmd.vx = vx;
                cmd.vy = vy;
                cmd.vz = vz;
                cmd.yaw_rate = 0.0f;
                cmd.priority = data_types::VelocityCommand::HIGH;
                cmd.source = data_types::VelocityCommand::OBSTACLE_AVOIDANCE;
                cmd.timestamp_us = now_us;
            }

            // Publish
            if (session.publish(reactive_vel_key, cmd.serialize())) {
                cmd_count++;
                if (cmd_count % static_cast<int>(config.control_rate) == 0) {
                    std::cout << "[Avoidance] state=" << state_name(state)
                              << " vel=(" << vx << ", " << vy << ", " << vz << ")"
                              << " min_dist=" << prox.min_distance() << "m" << std::endl;
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
