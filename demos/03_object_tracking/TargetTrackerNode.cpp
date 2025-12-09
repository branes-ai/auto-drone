// TargetTrackerNode.cpp
// Subscribes to object detections and odometry, computes velocity commands
// to track the detected object using visual servoing.

#include "ZenohInterface.hpp"
#include "Odometry.hpp"
#include "ObjectDetection.hpp"
#include "VelocityCommand.hpp"
#include "TargetTracker.hpp"

#include <iostream>
#include <chrono>
#include <thread>
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

// State protected by mutex
std::mutex g_state_mutex;
data_types::ObjectDetectionList g_detections;
data_types::Odometry g_odometry;
bool g_detections_received = false;
bool g_odometry_received = false;
uint64_t g_last_detection_time = 0;

// Configuration
struct Config {
    float target_distance = 3.0f;       // Desired distance to target (m)
    float max_forward_velocity = 2.0f;  // m/s
    float max_lateral_velocity = 1.0f;  // m/s
    float max_vertical_velocity = 1.0f; // m/s
    float max_yaw_rate = 1.0f;          // rad/s
    float control_rate = 50.0f;         // Hz
    float detection_timeout = 0.5f;     // seconds
    int image_width = 640;
    int image_height = 480;
    float kp_image = 1.5f;
    float kd_image = 0.2f;
    float kp_distance = 0.6f;
    float ki_distance = 0.05f;
};

void print_usage() {
    std::cout << "Usage: target_tracker_node [options]\n"
              << "Options:\n"
              << "  --target-dist <val>   Target distance in meters (default: 3.0)\n"
              << "  --max-vel <val>       Max forward velocity m/s (default: 2.0)\n"
              << "  --max-yaw <val>       Max yaw rate rad/s (default: 1.0)\n"
              << "  --rate <val>          Control rate Hz (default: 50)\n"
              << "  --timeout <val>       Detection timeout seconds (default: 0.5)\n"
              << "  --image-width <val>   Image width pixels (default: 640)\n"
              << "  --image-height <val>  Image height pixels (default: 480)\n"
              << std::endl;
}

void parse_args(int argc, char* argv[], Config& config) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage();
            std::exit(0);
        }
        if (arg == "--target-dist" && i + 1 < argc) config.target_distance = std::stof(argv[++i]);
        else if (arg == "--max-vel" && i + 1 < argc) config.max_forward_velocity = std::stof(argv[++i]);
        else if (arg == "--max-yaw" && i + 1 < argc) config.max_yaw_rate = std::stof(argv[++i]);
        else if (arg == "--rate" && i + 1 < argc) config.control_rate = std::stof(argv[++i]);
        else if (arg == "--timeout" && i + 1 < argc) config.detection_timeout = std::stof(argv[++i]);
        else if (arg == "--image-width" && i + 1 < argc) config.image_width = std::stoi(argv[++i]);
        else if (arg == "--image-height" && i + 1 < argc) config.image_height = std::stoi(argv[++i]);
    }
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    // Setup signal handling
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    Config config;
    parse_args(argc, argv, config);

    std::cout << "=== Target Tracker Node ===" << std::endl;
    std::cout << "Target distance: " << config.target_distance << " m" << std::endl;
    std::cout << "Max velocity: " << config.max_forward_velocity << " m/s" << std::endl;
    std::cout << "Max yaw rate: " << config.max_yaw_rate << " rad/s" << std::endl;
    std::cout << "Control rate: " << config.control_rate << " Hz" << std::endl;
    std::cout << "Detection timeout: " << config.detection_timeout << " s" << std::endl;
    std::cout << std::endl;

    // Initialize Zenoh session
    zenoh_interface::Session session;
    if (!session.is_valid()) {
        std::cerr << "Failed to create Zenoh session. Exiting." << std::endl;
        return 1;
    }

    // Setup target tracker
    control_algorithms::TargetTracker::Config tracker_config;
    tracker_config.image_width = config.image_width;
    tracker_config.image_height = config.image_height;
    tracker_config.target_distance = config.target_distance;
    tracker_config.image_x.gains = {config.kp_image, 0.0f, config.kd_image};
    tracker_config.image_y.gains = {config.kp_image, 0.0f, config.kd_image};
    tracker_config.distance.gains = {config.kp_distance, config.ki_distance, 0.1f};

    control_algorithms::TargetTracker tracker(tracker_config);

    control_algorithms::TargetTracker::Limits limits;
    limits.max_forward_velocity = config.max_forward_velocity;
    limits.max_lateral_velocity = config.max_lateral_velocity;
    limits.max_vertical_velocity = config.max_vertical_velocity;
    limits.max_yaw_rate = config.max_yaw_rate;
    tracker.set_limits(limits);

    // Zenoh key expressions
    const std::string detection_key = "robot/drone/perception/objects";
    const std::string odom_key = "robot/drone/sensor/state/odom";
    const std::string velocity_key = "robot/drone/cmd/velocity";

    // Subscribe to object detections
    session.subscribe(detection_key, [](const std::string&, const std::vector<uint8_t>& payload) {
        try {
            auto detections = data_types::ObjectDetectionList::deserialize(payload);
            std::lock_guard<std::mutex> lock(g_state_mutex);
            g_detections = detections;
            g_detections_received = true;
            g_last_detection_time = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        } catch (const std::exception& e) {
            std::cerr << "[Detection] Deserialization error: " << e.what() << std::endl;
        }
    });

    // Subscribe to odometry
    session.subscribe(odom_key, [](const std::string&, const std::vector<uint8_t>& payload) {
        try {
            auto odom = data_types::Odometry::deserialize(payload);
            std::lock_guard<std::mutex> lock(g_state_mutex);
            g_odometry = odom;
            g_odometry_received = true;
        } catch (const std::exception& e) {
            std::cerr << "[Odom] Deserialization error: " << e.what() << std::endl;
        }
    });

    std::cout << "Subscribed to:" << std::endl;
    std::cout << "  Detections: " << detection_key << std::endl;
    std::cout << "  Odometry: " << odom_key << std::endl;
    std::cout << "Publishing to:" << std::endl;
    std::cout << "  Velocity: " << velocity_key << std::endl;
    std::cout << std::endl;
    std::cout << "Waiting for detections..." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    // Control loop timing
    const auto control_interval = std::chrono::microseconds(
        static_cast<int>(1000000.0f / config.control_rate));
    auto last_control_time = std::chrono::steady_clock::now();
    int cmd_count = 0;
    bool tracking = false;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();

        if (now - last_control_time >= control_interval) {
            float dt = std::chrono::duration<float>(now - last_control_time).count();
            last_control_time = now;

            // Get current state
            data_types::ObjectDetectionList detections;
            bool have_target = false;
            float target_cx = 0, target_cy = 0, target_depth = 0;

            {
                std::lock_guard<std::mutex> lock(g_state_mutex);

                if (g_detections_received && !g_detections.empty()) {
                    // Check detection timeout
                    auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
                    float age = (now_us - g_last_detection_time) / 1000000.0f;

                    if (age < config.detection_timeout) {
                        // Track the largest (closest) detection
                        const data_types::ObjectDetection* best = nullptr;
                        float best_area = 0;

                        for (const auto& det : g_detections.detections) {
                            float area = det.bbox_area();
                            if (area > best_area) {
                                best_area = area;
                                best = &det;
                            }
                        }

                        if (best) {
                            target_cx = best->bbox_center_x();
                            target_cy = best->bbox_center_y();
                            if (best->has_3d_position) {
                                target_depth = std::sqrt(
                                    best->world_x * best->world_x +
                                    best->world_y * best->world_y +
                                    best->world_z * best->world_z);
                            } else {
                                target_depth = config.target_distance;
                            }
                            have_target = true;
                        }
                    }
                }
            }

            data_types::VelocityCommand cmd;
            cmd.source = data_types::VelocityCommand::WAYPOINT_MANAGER;
            cmd.priority = data_types::VelocityCommand::NORMAL;

            if (have_target) {
                if (!tracking) {
                    std::cout << "[Tracking] Target acquired!" << std::endl;
                    tracking = true;
                }

                // Compute tracking velocity
                float vx, vy, vz, yaw_rate;
                tracker.update(dt, target_cx, target_cy, target_depth,
                               vx, vy, vz, yaw_rate);

                cmd.vx = vx;
                cmd.vy = vy;
                cmd.vz = vz;
                cmd.yaw_rate = yaw_rate;
            } else {
                if (tracking) {
                    std::cout << "[Tracking] Target lost, hovering" << std::endl;
                    tracking = false;
                    tracker.reset();
                }

                // Hover when no target
                cmd.vx = 0;
                cmd.vy = 0;
                cmd.vz = 0;
                cmd.yaw_rate = 0;
            }

            cmd.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            if (session.publish(velocity_key, cmd.serialize())) {
                cmd_count++;
                if (cmd_count % static_cast<int>(config.control_rate) == 0 && tracking) {
                    std::cout << "[Tracking] target at (" << target_cx << ", " << target_cy
                              << ") depth=" << target_depth << "m"
                              << " -> vel=(" << cmd.vx << ", " << cmd.vy << ", " << cmd.vz
                              << ") yaw=" << cmd.yaw_rate << std::endl;
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
