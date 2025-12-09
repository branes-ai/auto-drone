// AirSimBridge.cpp
// Bridges AirSim simulator to Zenoh pub/sub network.
// - Publishes camera images and odometry from AirSim
// - Subscribes to velocity commands and forwards to AirSim

#include "AirSimClient.hpp"
#include "ZenohInterface.hpp"
#include "ImageData.hpp"
#include "Odometry.hpp"
#include "VelocityCommand.hpp"

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

// Current velocity command (protected by mutex)
std::mutex g_cmd_mutex;
data_types::VelocityCommand g_current_cmd;
bool g_cmd_received = false;
uint64_t g_last_cmd_time = 0;

// Configuration
struct Config {
    // AirSim connection
    std::string airsim_host = "localhost";
    int airsim_port = 41451;
    std::string vehicle_name = "Drone";

    // Camera settings
    std::string camera_name = "front_center";
    int image_width = 640;
    int image_height = 480;
    bool capture_depth = false;

    // Rates
    int image_fps = 30;
    int odom_rate = 100;  // Hz

    // Control
    float cmd_timeout = 0.5f;  // seconds - hover if no commands
    bool auto_takeoff = false;
    float takeoff_altitude = 10.0f;
};

void print_usage() {
    std::cout << "Usage: airsim_bridge [options]\n"
              << "Options:\n"
              << "  --host <ip>         AirSim host (default: localhost)\n"
              << "  --port <port>       AirSim port (default: 41451)\n"
              << "  --vehicle <name>    Vehicle name (default: Drone)\n"
              << "  --camera <name>     Camera name (default: front_center)\n"
              << "  --width <val>       Image width (default: 640)\n"
              << "  --height <val>      Image height (default: 480)\n"
              << "  --fps <val>         Image FPS (default: 30)\n"
              << "  --odom-rate <val>   Odometry rate Hz (default: 100)\n"
              << "  --depth             Enable depth capture\n"
              << "  --auto-takeoff      Automatically takeoff on start\n"
              << "  --altitude <val>    Takeoff altitude (default: 10.0)\n"
              << std::endl;
}

void parse_args(int argc, char* argv[], Config& config) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage();
            std::exit(0);
        }
        if (arg == "--host" && i + 1 < argc) config.airsim_host = argv[++i];
        else if (arg == "--port" && i + 1 < argc) config.airsim_port = std::stoi(argv[++i]);
        else if (arg == "--vehicle" && i + 1 < argc) config.vehicle_name = argv[++i];
        else if (arg == "--camera" && i + 1 < argc) config.camera_name = argv[++i];
        else if (arg == "--width" && i + 1 < argc) config.image_width = std::stoi(argv[++i]);
        else if (arg == "--height" && i + 1 < argc) config.image_height = std::stoi(argv[++i]);
        else if (arg == "--fps" && i + 1 < argc) config.image_fps = std::stoi(argv[++i]);
        else if (arg == "--odom-rate" && i + 1 < argc) config.odom_rate = std::stoi(argv[++i]);
        else if (arg == "--depth") config.capture_depth = true;
        else if (arg == "--auto-takeoff") config.auto_takeoff = true;
        else if (arg == "--altitude" && i + 1 < argc) config.takeoff_altitude = std::stof(argv[++i]);
    }
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    // Setup signal handling
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    Config config;
    parse_args(argc, argv, config);

    std::cout << "=== AirSim Bridge ===" << std::endl;
    std::cout << "AirSim: " << config.airsim_host << ":" << config.airsim_port << std::endl;
    std::cout << "Vehicle: " << config.vehicle_name << std::endl;
    std::cout << "Camera: " << config.camera_name << " @ " << config.image_width << "x"
              << config.image_height << " " << config.image_fps << " FPS" << std::endl;
    std::cout << "Odometry rate: " << config.odom_rate << " Hz" << std::endl;
    if (config.capture_depth) {
        std::cout << "Depth capture: enabled" << std::endl;
    }
    std::cout << std::endl;

    // Connect to AirSim
    std::cout << "Connecting to AirSim..." << std::endl;

    sim_interfaces::AirSimConfig airsim_config;
    airsim_config.host = config.airsim_host;
    airsim_config.port = config.airsim_port;
    airsim_config.vehicle_name = config.vehicle_name;
    airsim_config.camera_name = config.camera_name;
    airsim_config.image_width = config.image_width;
    airsim_config.image_height = config.image_height;
    airsim_config.capture_rgb = true;
    airsim_config.capture_depth = config.capture_depth;

    sim_interfaces::AirSimClient airsim(airsim_config);

    if (!airsim.connect()) {
        std::cerr << "Failed to connect to AirSim: " << airsim.last_error() << std::endl;
        std::cerr << "Make sure AirSim is running and accessible at "
                  << config.airsim_host << ":" << config.airsim_port << std::endl;
        return 1;
    }

    std::cout << "Connected to AirSim!" << std::endl;

    // Enable API control and arm
    if (!airsim.enable_api_control()) {
        std::cerr << "Failed to enable API control: " << airsim.last_error() << std::endl;
        return 1;
    }

    if (!airsim.arm()) {
        std::cerr << "Failed to arm: " << airsim.last_error() << std::endl;
        return 1;
    }

    std::cout << "API control enabled, drone armed" << std::endl;

    // Auto takeoff if requested
    if (config.auto_takeoff) {
        std::cout << "Taking off to " << config.takeoff_altitude << "m..." << std::endl;
        if (!airsim.takeoff()) {
            std::cerr << "Takeoff failed: " << airsim.last_error() << std::endl;
        } else {
            std::cout << "Takeoff complete!" << std::endl;
        }
    }

    // Initialize Zenoh session
    zenoh_interface::Session zenoh;
    if (!zenoh.is_valid()) {
        std::cerr << "Failed to create Zenoh session." << std::endl;
        return 1;
    }

    // Zenoh key expressions
    const std::string camera_rgb_key = "robot/drone/sensor/camera/rgb";
    const std::string camera_depth_key = "robot/drone/sensor/camera/depth";
    const std::string odom_key = "robot/drone/sensor/state/odom";
    const std::string velocity_key = "robot/drone/cmd/velocity";

    // Subscribe to velocity commands
    zenoh.subscribe(velocity_key, [](const std::string&, const std::vector<uint8_t>& payload) {
        try {
            auto cmd = data_types::VelocityCommand::deserialize(payload);
            std::lock_guard<std::mutex> lock(g_cmd_mutex);
            g_current_cmd = cmd;
            g_cmd_received = true;
            g_last_cmd_time = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        } catch (const std::exception& e) {
            std::cerr << "[Velocity] Deserialization error: " << e.what() << std::endl;
        }
    });

    std::cout << std::endl;
    std::cout << "Publishing to:" << std::endl;
    std::cout << "  RGB: " << camera_rgb_key << std::endl;
    if (config.capture_depth) {
        std::cout << "  Depth: " << camera_depth_key << std::endl;
    }
    std::cout << "  Odometry: " << odom_key << std::endl;
    std::cout << "Subscribed to:" << std::endl;
    std::cout << "  Velocity: " << velocity_key << std::endl;
    std::cout << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    // Timing
    auto start_time = std::chrono::steady_clock::now();
    auto last_image_time = start_time;
    auto last_odom_time = start_time;
    auto last_cmd_time = start_time;

    const auto image_interval = std::chrono::microseconds(1000000 / config.image_fps);
    const auto odom_interval = std::chrono::microseconds(1000000 / config.odom_rate);
    const auto cmd_interval = std::chrono::milliseconds(20);  // 50 Hz command rate

    int frame_count = 0;
    int odom_count = 0;
    int cmd_count = 0;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();

        // Capture and publish images
        if (now - last_image_time >= image_interval) {
            last_image_time = now;

            // RGB
            auto rgb = airsim.capture_rgb();
            if (rgb.is_valid()) {
                auto payload = rgb.serialize();
                if (zenoh.publish(camera_rgb_key, payload)) {
                    frame_count++;
                    if (frame_count % config.image_fps == 0) {
                        std::cout << "[Camera] Published frame " << frame_count
                                  << " (" << payload.size() << " bytes)" << std::endl;
                    }
                }
            }

            // Depth
            if (config.capture_depth) {
                auto depth = airsim.capture_depth();
                if (depth.is_valid()) {
                    zenoh.publish(camera_depth_key, depth.serialize());
                }
            }
        }

        // Publish odometry
        if (now - last_odom_time >= odom_interval) {
            last_odom_time = now;

            auto odom = airsim.get_odometry();
            auto payload = odom.serialize();
            if (zenoh.publish(odom_key, payload)) {
                odom_count++;
                if (odom_count % config.odom_rate == 0) {
                    std::cout << "[Odom] x=" << odom.x << " y=" << odom.y
                              << " z=" << odom.z << " yaw=" << odom.yaw << std::endl;
                }
            }
        }

        // Apply velocity commands
        if (now - last_cmd_time >= cmd_interval) {
            last_cmd_time = now;

            data_types::VelocityCommand cmd;
            bool apply_cmd = false;

            {
                std::lock_guard<std::mutex> lock(g_cmd_mutex);
                if (g_cmd_received) {
                    auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
                    float age = (now_us - g_last_cmd_time) / 1000000.0f;

                    if (age < config.cmd_timeout) {
                        cmd = g_current_cmd;
                        apply_cmd = true;
                    }
                }
            }

            if (apply_cmd) {
                if (airsim.apply_velocity_command(cmd)) {
                    cmd_count++;
                }
            } else {
                // No recent command - hover
                airsim.move_by_velocity(0, 0, 0);
            }
        }

        // Small sleep to prevent busy-waiting
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    std::cout << std::endl;
    std::cout << "Shutting down..." << std::endl;

    // Hover and land
    std::cout << "Hovering..." << std::endl;
    airsim.hover();

    std::cout << "Disabling API control..." << std::endl;
    airsim.disable_api_control();

    std::cout << std::endl;
    std::cout << "Statistics:" << std::endl;
    std::cout << "  Frames published: " << frame_count << std::endl;
    std::cout << "  Odometry messages: " << odom_count << std::endl;
    std::cout << "  Commands applied: " << cmd_count << std::endl;

    return 0;
}
