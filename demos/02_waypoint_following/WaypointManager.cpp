// WaypointManager.cpp
// Subscribes to odometry and waypoint lists, runs PID control loop,
// and publishes velocity commands to reach waypoints in sequence.

#include "ZenohInterface.hpp"
#include "Odometry.hpp"
#include "Waypoint.hpp"
#include "VelocityCommand.hpp"
#include "PIDController.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <csignal>
#include <atomic>
#include <mutex>
#include <string>

namespace {

std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
}

// Current state protected by mutex
std::mutex g_state_mutex;
data_types::Odometry g_current_odom;
data_types::WaypointList g_waypoints;
size_t g_current_waypoint_index = 0;
bool g_odom_received = false;
bool g_waypoints_received = false;

// Configuration
struct Config {
    float position_kp = 0.8f;
    float position_ki = 0.05f;
    float position_kd = 0.2f;
    float yaw_kp = 1.0f;
    float yaw_ki = 0.0f;
    float yaw_kd = 0.1f;
    float max_velocity = 2.0f;    // m/s
    float max_yaw_rate = 1.0f;    // rad/s
    float waypoint_radius = 0.5f; // m - distance to consider waypoint reached
    float control_rate = 50.0f;   // Hz
};

// Parse command line arguments
void parse_args(int argc, char* argv[], Config& config) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--help" || arg == "-h")) {
            std::cout << "Usage: waypoint_manager [options]\n"
                      << "Options:\n"
                      << "  --pos-kp <val>    Position P gain (default: 0.8)\n"
                      << "  --pos-ki <val>    Position I gain (default: 0.05)\n"
                      << "  --pos-kd <val>    Position D gain (default: 0.2)\n"
                      << "  --yaw-kp <val>    Yaw P gain (default: 1.0)\n"
                      << "  --max-vel <val>   Max velocity m/s (default: 2.0)\n"
                      << "  --radius <val>    Waypoint reach radius m (default: 0.5)\n"
                      << "  --rate <val>      Control loop rate Hz (default: 50)\n"
                      << std::endl;
            std::exit(0);
        }
        if (arg == "--pos-kp" && i + 1 < argc) config.position_kp = std::stof(argv[++i]);
        else if (arg == "--pos-ki" && i + 1 < argc) config.position_ki = std::stof(argv[++i]);
        else if (arg == "--pos-kd" && i + 1 < argc) config.position_kd = std::stof(argv[++i]);
        else if (arg == "--yaw-kp" && i + 1 < argc) config.yaw_kp = std::stof(argv[++i]);
        else if (arg == "--max-vel" && i + 1 < argc) config.max_velocity = std::stof(argv[++i]);
        else if (arg == "--radius" && i + 1 < argc) config.waypoint_radius = std::stof(argv[++i]);
        else if (arg == "--rate" && i + 1 < argc) config.control_rate = std::stof(argv[++i]);
    }
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    // Setup signal handling
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    Config config;
    parse_args(argc, argv, config);

    std::cout << "=== Waypoint Manager ===" << std::endl;
    std::cout << "Position PID: kp=" << config.position_kp
              << " ki=" << config.position_ki
              << " kd=" << config.position_kd << std::endl;
    std::cout << "Yaw PID: kp=" << config.yaw_kp << std::endl;
    std::cout << "Max velocity: " << config.max_velocity << " m/s" << std::endl;
    std::cout << "Waypoint radius: " << config.waypoint_radius << " m" << std::endl;
    std::cout << "Control rate: " << config.control_rate << " Hz" << std::endl;
    std::cout << std::endl;

    // Initialize Zenoh session
    zenoh_interface::Session session;
    if (!session.is_valid()) {
        std::cerr << "Failed to create Zenoh session. Exiting." << std::endl;
        return 1;
    }

    // Zenoh key expressions
    const std::string odom_key = "robot/drone/sensor/state/odom";
    const std::string waypoint_key = "robot/drone/cmd/waypoints";
    const std::string velocity_key = "robot/drone/cmd/velocity";

    // Subscribe to odometry
    session.subscribe(odom_key, [](const std::string&, const std::vector<uint8_t>& payload) {
        try {
            auto odom = data_types::Odometry::deserialize(payload);
            std::lock_guard<std::mutex> lock(g_state_mutex);
            g_current_odom = odom;
            g_odom_received = true;
        } catch (const std::exception& e) {
            std::cerr << "[Odom] Deserialization error: " << e.what() << std::endl;
        }
    });

    // Subscribe to waypoint commands
    session.subscribe(waypoint_key, [](const std::string&, const std::vector<uint8_t>& payload) {
        try {
            auto waypoints = data_types::WaypointList::deserialize(payload);
            std::lock_guard<std::mutex> lock(g_state_mutex);
            g_waypoints = waypoints;
            g_current_waypoint_index = 0;
            g_waypoints_received = true;
            std::cout << "[Waypoints] Received " << waypoints.size() << " waypoints" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[Waypoints] Deserialization error: " << e.what() << std::endl;
        }
    });

    std::cout << "Subscribed to:" << std::endl;
    std::cout << "  Odometry: " << odom_key << std::endl;
    std::cout << "  Waypoints: " << waypoint_key << std::endl;
    std::cout << "Publishing to:" << std::endl;
    std::cout << "  Velocity: " << velocity_key << std::endl;
    std::cout << std::endl;
    std::cout << "Waiting for waypoints and odometry..." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    // Setup PID controllers
    control_algorithms::PositionController position_controller(
        config.position_kp, config.position_ki, config.position_kd);
    control_algorithms::YawController yaw_controller(
        config.yaw_kp, config.yaw_ki, config.yaw_kd);

    // Set velocity limits
    control_algorithms::PIDController::Limits pos_limits;
    pos_limits.output_min = -config.max_velocity;
    pos_limits.output_max = config.max_velocity;
    pos_limits.integral_min = -config.max_velocity * 2.0f;
    pos_limits.integral_max = config.max_velocity * 2.0f;
    position_controller.x().set_limits(pos_limits);
    position_controller.y().set_limits(pos_limits);
    position_controller.z().set_limits(pos_limits);

    control_algorithms::PIDController::Limits yaw_limits;
    yaw_limits.output_min = -config.max_yaw_rate;
    yaw_limits.output_max = config.max_yaw_rate;
    yaw_controller.controller().set_limits(yaw_limits);

    // Control loop timing
    const auto control_interval = std::chrono::microseconds(
        static_cast<int>(1000000.0f / config.control_rate));
    auto last_control_time = std::chrono::steady_clock::now();
    int cmd_count = 0;
    bool mission_complete = false;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();

        if (now - last_control_time >= control_interval) {
            float dt = std::chrono::duration<float>(now - last_control_time).count();
            last_control_time = now;

            // Get current state
            data_types::Odometry odom;
            data_types::Waypoint target_wp;
            bool have_target = false;
            bool reached_waypoint = false;

            {
                std::lock_guard<std::mutex> lock(g_state_mutex);
                if (!g_odom_received || !g_waypoints_received || g_waypoints.empty()) {
                    continue;
                }
                if (g_current_waypoint_index >= g_waypoints.size()) {
                    if (!mission_complete) {
                        std::cout << "[Mission] All waypoints reached!" << std::endl;
                        mission_complete = true;
                    }
                    // Publish hover command
                    auto hover_cmd = data_types::VelocityCommand::hover(
                        data_types::VelocityCommand::LOW,
                        data_types::VelocityCommand::WAYPOINT_MANAGER);
                    session.publish(velocity_key, hover_cmd.serialize());
                    continue;
                }
                odom = g_current_odom;
                target_wp = g_waypoints[g_current_waypoint_index];
                have_target = true;

                // Check if waypoint is reached
                float dx = target_wp.x - odom.x;
                float dy = target_wp.y - odom.y;
                float dz = target_wp.z - odom.z;
                float distance = std::sqrt(dx*dx + dy*dy + dz*dz);

                if (distance < config.waypoint_radius) {
                    std::cout << "[Waypoint " << target_wp.id << "] Reached at distance "
                              << distance << "m" << std::endl;
                    g_current_waypoint_index++;
                    position_controller.reset();
                    yaw_controller.reset();
                    reached_waypoint = true;
                    mission_complete = false;
                }
            }

            if (!have_target || reached_waypoint) {
                continue;
            }

            // Compute velocity commands using PID
            float vx, vy, vz;
            position_controller.update(dt,
                target_wp.x, target_wp.y, target_wp.z,
                odom.x, odom.y, odom.z,
                vx, vy, vz);

            // Compute yaw rate if HOLD_YAW flag is set
            float yaw_rate = 0.0f;
            if (target_wp.flags & data_types::Waypoint::HOLD_YAW) {
                yaw_rate = yaw_controller.update(dt, target_wp.yaw, odom.yaw);
            }

            // Create and publish velocity command
            auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            data_types::VelocityCommand cmd(
                vx, vy, vz, yaw_rate,
                data_types::VelocityCommand::LOW,
                data_types::VelocityCommand::WAYPOINT_MANAGER,
                now_us);

            if (session.publish(velocity_key, cmd.serialize())) {
                cmd_count++;
                if (cmd_count % static_cast<int>(config.control_rate) == 0) {
                    float dx = target_wp.x - odom.x;
                    float dy = target_wp.y - odom.y;
                    float dz = target_wp.z - odom.z;
                    float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                    std::cout << "[WP " << target_wp.id << "] dist=" << distance
                              << "m vel=(" << vx << ", " << vy << ", " << vz << ")" << std::endl;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    std::cout << std::endl;
    std::cout << "Shutting down..." << std::endl;
    std::cout << "Total velocity commands: " << cmd_count << std::endl;

    return 0;
}
