// MockProximityPublisher.cpp
// Generates synthetic proximity sensor data for testing obstacle avoidance.
// Simulates obstacles approaching the drone from configurable directions.

#include "ZenohInterface.hpp"
#include "ProximityData.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <string>
#include <cmath>

namespace {

std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
}

enum class Direction {
    FRONT,
    BACK,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

struct Config {
    Direction obstacle_direction = Direction::FRONT;
    float initial_distance = 10.0f;     // Starting distance (m)
    float min_distance = 0.3f;          // Closest approach (m)
    float approach_speed = 0.5f;        // How fast obstacle approaches (m/s)
    float publish_rate = 20.0f;         // Hz
    bool oscillate = false;             // If true, obstacle moves back and forth
    std::string connect_endpoint;       // Remote Zenoh endpoint
};

void print_usage() {
    std::cout << "Usage: mock_proximity_publisher [options]\n"
              << "Options:\n"
              << "  --direction <dir>    Obstacle direction: front|back|left|right|up|down (default: front)\n"
              << "  --initial <dist>     Initial distance in meters (default: 10.0)\n"
              << "  --min-dist <dist>    Minimum distance in meters (default: 0.3)\n"
              << "  --speed <val>        Approach speed in m/s (default: 0.5)\n"
              << "  --rate <val>         Publish rate in Hz (default: 20)\n"
              << "  --oscillate          Obstacle moves back and forth\n"
              << "  --connect <ep>       Remote Zenoh endpoint (e.g., tcp/192.168.1.10:7447)\n"
              << std::endl;
}

Direction parse_direction(const std::string& s) {
    if (s == "front") return Direction::FRONT;
    if (s == "back") return Direction::BACK;
    if (s == "left") return Direction::LEFT;
    if (s == "right") return Direction::RIGHT;
    if (s == "up") return Direction::UP;
    if (s == "down") return Direction::DOWN;
    std::cerr << "Unknown direction: " << s << ", using front" << std::endl;
    return Direction::FRONT;
}

std::string direction_name(Direction d) {
    switch (d) {
        case Direction::FRONT: return "front";
        case Direction::BACK: return "back";
        case Direction::LEFT: return "left";
        case Direction::RIGHT: return "right";
        case Direction::UP: return "up";
        case Direction::DOWN: return "down";
    }
    return "unknown";
}

void parse_args(int argc, char* argv[], Config& config) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage();
            std::exit(0);
        }
        if (arg == "--direction" && i + 1 < argc) config.obstacle_direction = parse_direction(argv[++i]);
        else if (arg == "--initial" && i + 1 < argc) config.initial_distance = std::stof(argv[++i]);
        else if (arg == "--min-dist" && i + 1 < argc) config.min_distance = std::stof(argv[++i]);
        else if (arg == "--speed" && i + 1 < argc) config.approach_speed = std::stof(argv[++i]);
        else if (arg == "--rate" && i + 1 < argc) config.publish_rate = std::stof(argv[++i]);
        else if (arg == "--oscillate") config.oscillate = true;
        else if (arg == "--connect" && i + 1 < argc) config.connect_endpoint = argv[++i];
    }
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    Config config;
    parse_args(argc, argv, config);

    std::cout << "=== Mock Proximity Publisher ===" << std::endl;
    std::cout << "Obstacle direction: " << direction_name(config.obstacle_direction) << std::endl;
    std::cout << "Initial distance: " << config.initial_distance << " m" << std::endl;
    std::cout << "Min distance: " << config.min_distance << " m" << std::endl;
    std::cout << "Approach speed: " << config.approach_speed << " m/s" << std::endl;
    std::cout << "Publish rate: " << config.publish_rate << " Hz" << std::endl;
    std::cout << "Oscillate: " << (config.oscillate ? "yes" : "no") << std::endl;
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

    const std::string proximity_key = "robot/drone/sensor/proximity";
    std::cout << "Publishing to: " << proximity_key << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;
    std::cout << std::endl;

    // Timing
    const auto publish_interval = std::chrono::microseconds(
        static_cast<int>(1000000.0f / config.publish_rate));
    auto last_publish_time = std::chrono::steady_clock::now();
    auto start_time = std::chrono::steady_clock::now();

    float current_distance = config.initial_distance;
    bool approaching = true;  // Direction for oscillation mode
    int pub_count = 0;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();

        if (now - last_publish_time >= publish_interval) {
            float dt = std::chrono::duration<float>(now - last_publish_time).count();
            last_publish_time = now;

            // Update obstacle distance
            if (config.oscillate) {
                if (approaching) {
                    current_distance -= config.approach_speed * dt;
                    if (current_distance <= config.min_distance) {
                        current_distance = config.min_distance;
                        approaching = false;
                    }
                } else {
                    current_distance += config.approach_speed * dt;
                    if (current_distance >= config.initial_distance) {
                        current_distance = config.initial_distance;
                        approaching = true;
                    }
                }
            } else {
                // One-way approach
                current_distance -= config.approach_speed * dt;
                if (current_distance < config.min_distance) {
                    current_distance = config.min_distance;
                }
            }

            // Create proximity data
            data_types::ProximityData prox = data_types::ProximityData::clear();
            prox.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            // Set the obstacle in the configured direction
            switch (config.obstacle_direction) {
                case Direction::FRONT: prox.distance_front = current_distance; break;
                case Direction::BACK: prox.distance_back = current_distance; break;
                case Direction::LEFT: prox.distance_left = current_distance; break;
                case Direction::RIGHT: prox.distance_right = current_distance; break;
                case Direction::UP: prox.distance_up = current_distance; break;
                case Direction::DOWN: prox.distance_down = current_distance; break;
            }

            // Publish
            if (session.publish(proximity_key, prox.serialize())) {
                pub_count++;
                if (pub_count % static_cast<int>(config.publish_rate) == 0) {
                    std::cout << "[Proximity] " << direction_name(config.obstacle_direction)
                              << "=" << current_distance << "m" << std::endl;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    std::cout << std::endl;
    std::cout << "Shutting down..." << std::endl;
    std::cout << "Total publishes: " << pub_count << std::endl;

    return 0;
}
