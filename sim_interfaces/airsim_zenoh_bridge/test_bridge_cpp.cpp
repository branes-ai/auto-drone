/**
 * C++ test program for verifying AirSim Zenoh Bridge connectivity.
 *
 * Run this on the Linux workstation to test connectivity with the
 * bridge running on the Windows workstation.
 *
 * Usage:
 *   ./test_bridge_cpp --connect tcp/192.168.1.10:7447
 *   ./test_bridge_cpp --connect tcp/192.168.1.10:7447 --send-commands
 */

#include <iostream>
#include <atomic>
#include <chrono>
#include <thread>
#include <csignal>
#include <cstring>

#include "ZenohInterface.hpp"
#include "ImageData.hpp"
#include "Odometry.hpp"
#include "VelocityCommand.hpp"

using namespace zenoh_interface;
using namespace data_types;

// Global flag for signal handling
std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
}

struct Stats {
    std::atomic<uint64_t> rgb_frames{0};
    std::atomic<uint64_t> depth_frames{0};
    std::atomic<uint64_t> odom_messages{0};
    Odometry last_odom{};
    std::pair<int, int> last_rgb_size{0, 0};
    std::pair<int, int> last_depth_size{0, 0};
};

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " --connect <endpoint> [options]\n"
              << "\nOptions:\n"
              << "  --connect <endpoint>  Zenoh endpoint (e.g., tcp/192.168.1.10:7447)\n"
              << "  --robot-id <id>       Robot ID for topics (default: drone)\n"
              << "  --duration <sec>      Test duration in seconds (default: 10)\n"
              << "  --send-commands       Send test velocity commands\n"
              << "  --help                Show this help\n";
}

int main(int argc, char* argv[]) {
    // Parse arguments
    std::string connect_endpoint;
    std::string robot_id = "drone";
    float duration = 10.0f;
    bool send_commands = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--connect") == 0 && i + 1 < argc) {
            connect_endpoint = argv[++i];
        } else if (std::strcmp(argv[i], "--robot-id") == 0 && i + 1 < argc) {
            robot_id = argv[++i];
        } else if (std::strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
            duration = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--send-commands") == 0) {
            send_commands = true;
        } else if (std::strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    if (connect_endpoint.empty()) {
        std::cerr << "Error: --connect endpoint is required\n\n";
        print_usage(argv[0]);
        return 1;
    }

    // Setup signal handler
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Create Zenoh session with remote endpoint
    std::cout << "Connecting to Zenoh bridge at " << connect_endpoint << "...\n";

    SessionConfig config = SessionConfig::connect_to(connect_endpoint);
    Session session(config);

    if (!session.is_valid()) {
        std::cerr << "Failed to create Zenoh session\n";
        return 1;
    }

    // Topic key expressions
    std::string topic_rgb = "robot/" + robot_id + "/sensor/camera/rgb";
    std::string topic_depth = "robot/" + robot_id + "/sensor/camera/depth";
    std::string topic_odom = "robot/" + robot_id + "/sensor/state/odom";
    std::string topic_cmd = "robot/" + robot_id + "/cmd/velocity";

    Stats stats;

    // Subscribe to RGB
    session.subscribe(topic_rgb, [&stats](const std::string&, const std::vector<uint8_t>& payload) {
        try {
            auto img = ImageData::deserialize(payload);
            stats.rgb_frames++;
            stats.last_rgb_size = {static_cast<int>(img.width), static_cast<int>(img.height)};
        } catch (const std::exception& e) {
            std::cerr << "Error parsing RGB: " << e.what() << "\n";
        }
    });

    // Subscribe to Depth
    session.subscribe(topic_depth, [&stats](const std::string&, const std::vector<uint8_t>& payload) {
        try {
            auto img = ImageData::deserialize(payload);
            stats.depth_frames++;
            stats.last_depth_size = {static_cast<int>(img.width), static_cast<int>(img.height)};
        } catch (const std::exception& e) {
            std::cerr << "Error parsing depth: " << e.what() << "\n";
        }
    });

    // Subscribe to Odometry
    session.subscribe(topic_odom, [&stats](const std::string&, const std::vector<uint8_t>& payload) {
        try {
            stats.last_odom = Odometry::deserialize(payload);
            stats.odom_messages++;
        } catch (const std::exception& e) {
            std::cerr << "Error parsing odometry: " << e.what() << "\n";
        }
    });

    std::cout << "\nSubscribed to topics:\n"
              << "  RGB:   " << topic_rgb << "\n"
              << "  Depth: " << topic_depth << "\n"
              << "  Odom:  " << topic_odom << "\n\n";

    std::cout << "Listening for " << duration << " seconds...\n";
    if (send_commands) {
        std::cout << "Will send test commands every 2 seconds\n";
    }

    auto start_time = std::chrono::steady_clock::now();
    auto last_command_time = start_time;
    int cmd_idx = 0;

    // Command sequence: forward, right, backward, left, rotate, hover
    float commands[][4] = {
        {0.5f, 0.0f, 0.0f, 0.0f},   // Forward
        {0.0f, 0.5f, 0.0f, 0.0f},   // Right
        {-0.5f, 0.0f, 0.0f, 0.0f},  // Backward
        {0.0f, -0.5f, 0.0f, 0.0f},  // Left
        {0.0f, 0.0f, 0.0f, 0.5f},   // Rotate CW
        {0.0f, 0.0f, 0.0f, 0.0f},   // Hover
    };
    const int num_commands = 6;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<float>(now - start_time).count();

        if (elapsed >= duration) {
            break;
        }

        // Send test commands if enabled
        if (send_commands) {
            auto cmd_elapsed = std::chrono::duration<float>(now - last_command_time).count();
            if (cmd_elapsed >= 2.0f) {
                float* cmd = commands[cmd_idx % num_commands];
                VelocityCommand vel_cmd(
                    cmd[0], cmd[1], cmd[2], cmd[3],
                    VelocityCommand::Priority::NORMAL,
                    VelocityCommand::Source::MANUAL_CONTROL
                );

                auto payload = vel_cmd.serialize();
                session.publish(topic_cmd, payload);

                std::cout << "Sent command: vx=" << cmd[0]
                          << ", vy=" << cmd[1]
                          << ", vz=" << cmd[2]
                          << ", yaw=" << cmd[3] << "\n";

                cmd_idx++;
                last_command_time = now;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Print final statistics
    auto end_time = std::chrono::steady_clock::now();
    float total_elapsed = std::chrono::duration<float>(end_time - start_time).count();

    std::cout << "\n============================================================\n";
    std::cout << "Bridge Test Statistics (elapsed: " << total_elapsed << "s)\n";
    std::cout << "============================================================\n";

    float rgb_rate = stats.rgb_frames / total_elapsed;
    float depth_rate = stats.depth_frames / total_elapsed;
    float odom_rate = stats.odom_messages / total_elapsed;

    std::cout << "RGB Frames:    " << stats.rgb_frames << "  ("
              << rgb_rate << " Hz)  Size: "
              << stats.last_rgb_size.first << "x" << stats.last_rgb_size.second << "\n";
    std::cout << "Depth Frames:  " << stats.depth_frames << "  ("
              << depth_rate << " Hz)  Size: "
              << stats.last_depth_size.first << "x" << stats.last_depth_size.second << "\n";
    std::cout << "Odom Messages: " << stats.odom_messages << "  ("
              << odom_rate << " Hz)\n";

    if (stats.odom_messages > 0) {
        const auto& o = stats.last_odom;
        std::cout << "\nLast Odometry:\n";
        std::cout << "  Position: x=" << o.x << ", y=" << o.y << ", z=" << o.z << "\n";
        std::cout << "  Rotation: roll=" << o.roll << ", pitch=" << o.pitch << ", yaw=" << o.yaw << "\n";
    }

    if (stats.odom_messages > 0) {
        std::cout << "\n✓ Bridge connection: OK\n";
        return 0;
    } else {
        std::cout << "\n✗ Bridge connection: NO DATA RECEIVED\n";
        std::cout << "  Check that:\n";
        std::cout << "  - The bridge is running on Windows\n";
        std::cout << "  - Project AirSim is running\n";
        std::cout << "  - Firewall port 7447 is open\n";
        std::cout << "  - The endpoint address is correct\n";
        return 1;
    }
}
