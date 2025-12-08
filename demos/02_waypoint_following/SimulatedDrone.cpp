// SimulatedDrone.cpp
// A simple simulated drone that:
// - Subscribes to velocity commands
// - Integrates the velocity to update position
// - Publishes odometry at high rate
// This allows testing the waypoint manager without a real simulator.

#include "ZenohInterface.hpp"
#include "Odometry.hpp"
#include "VelocityCommand.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <csignal>
#include <atomic>
#include <mutex>
#include <string>
#include <algorithm>

namespace {

std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
}

// Current state protected by mutex
std::mutex g_state_mutex;
data_types::VelocityCommand g_current_cmd;
bool g_cmd_received = false;
uint64_t g_last_cmd_time_us = 0;

// Drone state
struct DroneState {
    float x = 0.0f;
    float y = 0.0f;
    float z = -10.0f;  // Start at 10m altitude (NED: negative is up)
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
};

// Configuration
struct Config {
    float odom_rate = 100.0f;   // Hz
    float sim_rate = 200.0f;    // Hz - simulation update rate
    float cmd_timeout = 0.5f;   // seconds - stop if no commands received
    float damping = 2.0f;       // velocity damping factor
    float max_tilt = 0.3f;      // radians - max roll/pitch during acceleration
};

void parse_args(int argc, char* argv[], Config& config, DroneState& state) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: simulated_drone [options]\n"
                      << "Options:\n"
                      << "  --odom-rate <val>   Odometry publish rate Hz (default: 100)\n"
                      << "  --sim-rate <val>    Simulation rate Hz (default: 200)\n"
                      << "  --start-x <val>     Starting X position (default: 0)\n"
                      << "  --start-y <val>     Starting Y position (default: 0)\n"
                      << "  --start-z <val>     Starting Z position (default: -10, NED)\n"
                      << std::endl;
            std::exit(0);
        }
        if (arg == "--odom-rate" && i + 1 < argc) config.odom_rate = std::stof(argv[++i]);
        else if (arg == "--sim-rate" && i + 1 < argc) config.sim_rate = std::stof(argv[++i]);
        else if (arg == "--start-x" && i + 1 < argc) state.x = std::stof(argv[++i]);
        else if (arg == "--start-y" && i + 1 < argc) state.y = std::stof(argv[++i]);
        else if (arg == "--start-z" && i + 1 < argc) state.z = std::stof(argv[++i]);
    }
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    // Setup signal handling
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    Config config;
    DroneState state;
    parse_args(argc, argv, config, state);

    std::cout << "=== Simulated Drone ===" << std::endl;
    std::cout << "Starting position: (" << state.x << ", " << state.y << ", " << state.z << ")" << std::endl;
    std::cout << "Odometry rate: " << config.odom_rate << " Hz" << std::endl;
    std::cout << "Simulation rate: " << config.sim_rate << " Hz" << std::endl;
    std::cout << std::endl;

    // Initialize Zenoh session
    zenoh_interface::Session session;
    if (!session.is_valid()) {
        std::cerr << "Failed to create Zenoh session. Exiting." << std::endl;
        return 1;
    }

    // Zenoh key expressions
    const std::string velocity_key = "robot/drone/cmd/velocity";
    const std::string odom_key = "robot/drone/sensor/state/odom";

    // Subscribe to velocity commands
    session.subscribe(velocity_key, [](const std::string&, const std::vector<uint8_t>& payload) {
        try {
            auto cmd = data_types::VelocityCommand::deserialize(payload);
            std::lock_guard<std::mutex> lock(g_state_mutex);
            g_current_cmd = cmd;
            g_cmd_received = true;
            g_last_cmd_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        } catch (const std::exception& e) {
            std::cerr << "[Cmd] Deserialization error: " << e.what() << std::endl;
        }
    });

    std::cout << "Subscribed to: " << velocity_key << std::endl;
    std::cout << "Publishing to: " << odom_key << std::endl;
    std::cout << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    // Timing
    const auto sim_interval = std::chrono::microseconds(
        static_cast<int>(1000000.0f / config.sim_rate));
    const auto odom_interval = std::chrono::microseconds(
        static_cast<int>(1000000.0f / config.odom_rate));

    auto last_sim_time = std::chrono::steady_clock::now();
    auto last_odom_time = last_sim_time;
    int odom_count = 0;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();

        // Simulation update
        if (now - last_sim_time >= sim_interval) {
            float dt = std::chrono::duration<float>(now - last_sim_time).count();
            last_sim_time = now;

            // Get current velocity command
            data_types::VelocityCommand cmd;
            bool have_cmd = false;
            {
                std::lock_guard<std::mutex> lock(g_state_mutex);
                if (g_cmd_received) {
                    auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
                    float cmd_age = (now_us - g_last_cmd_time_us) / 1000000.0f;

                    if (cmd_age < config.cmd_timeout) {
                        cmd = g_current_cmd;
                        have_cmd = true;
                    }
                }
            }

            // Apply velocity command with damping
            float target_vx = have_cmd ? cmd.vx : 0.0f;
            float target_vy = have_cmd ? cmd.vy : 0.0f;
            float target_vz = have_cmd ? cmd.vz : 0.0f;
            float yaw_rate = have_cmd ? cmd.yaw_rate : 0.0f;

            // Simple first-order response to velocity commands
            float alpha = 1.0f - std::exp(-config.damping * dt);
            state.vx += alpha * (target_vx - state.vx);
            state.vy += alpha * (target_vy - state.vy);
            state.vz += alpha * (target_vz - state.vz);

            // Integrate position
            state.x += state.vx * dt;
            state.y += state.vy * dt;
            state.z += state.vz * dt;

            // Integrate yaw
            state.yaw += yaw_rate * dt;
            // Wrap yaw to [-PI, PI]
            constexpr float PI = 3.14159265358979323846f;
            while (state.yaw > PI) state.yaw -= 2.0f * PI;
            while (state.yaw < -PI) state.yaw += 2.0f * PI;

            // Simulate roll/pitch based on acceleration (simple approximation)
            state.roll = -state.vy * 0.1f;
            state.pitch = state.vx * 0.1f;
            state.roll = std::clamp(state.roll, -config.max_tilt, config.max_tilt);
            state.pitch = std::clamp(state.pitch, -config.max_tilt, config.max_tilt);
        }

        // Publish odometry
        if (now - last_odom_time >= odom_interval) {
            last_odom_time = now;

            auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            data_types::Odometry odom(
                state.x, state.y, state.z,
                state.roll, state.pitch, state.yaw,
                now_us);

            if (session.publish(odom_key, odom.serialize())) {
                odom_count++;
                if (odom_count % static_cast<int>(config.odom_rate) == 0) {
                    std::cout << "[Odom] pos=(" << state.x << ", " << state.y << ", " << state.z
                              << ") vel=(" << state.vx << ", " << state.vy << ", " << state.vz
                              << ") yaw=" << state.yaw << std::endl;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    std::cout << std::endl;
    std::cout << "Shutting down..." << std::endl;
    std::cout << "Final position: (" << state.x << ", " << state.y << ", " << state.z << ")" << std::endl;
    std::cout << "Total odometry messages: " << odom_count << std::endl;

    return 0;
}
