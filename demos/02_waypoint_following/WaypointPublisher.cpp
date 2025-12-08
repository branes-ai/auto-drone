// WaypointPublisher.cpp
// Simple utility to publish a waypoint sequence for testing the waypoint manager.
// Reads waypoints from command line or uses a default square pattern.

#include "ZenohInterface.hpp"
#include "Waypoint.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <cmath>

namespace {

constexpr float PI = 3.14159265358979323846f;

// Create a square pattern of waypoints
data_types::WaypointList create_square_pattern(float size, float altitude, float speed) {
    data_types::WaypointList list;

    // Square pattern: start at origin, go to each corner
    list.add(data_types::Waypoint(size, 0.0f, -altitude, 0.0f, speed, 0,
        data_types::Waypoint::HOLD_YAW));
    list.add(data_types::Waypoint(size, size, -altitude, PI/2, speed, 1,
        data_types::Waypoint::HOLD_YAW));
    list.add(data_types::Waypoint(0.0f, size, -altitude, PI, speed, 2,
        data_types::Waypoint::HOLD_YAW));
    list.add(data_types::Waypoint(0.0f, 0.0f, -altitude, -PI/2, speed, 3,
        data_types::Waypoint::HOLD_YAW | data_types::Waypoint::FINAL));

    auto now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    list.timestamp_us = now;

    return list;
}

// Create a circle pattern of waypoints
data_types::WaypointList create_circle_pattern(float radius, float altitude, float speed, int points) {
    data_types::WaypointList list;

    for (int i = 0; i < points; ++i) {
        float angle = 2.0f * PI * i / points;
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);
        uint8_t flags = data_types::Waypoint::HOLD_YAW;
        if (i == points - 1) {
            flags |= data_types::Waypoint::FINAL;
        }
        list.add(data_types::Waypoint(x, y, -altitude, angle + PI/2, speed, i, flags));
    }

    auto now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    list.timestamp_us = now;

    return list;
}

void print_usage() {
    std::cout << "Usage: waypoint_publisher [options]\n"
              << "Options:\n"
              << "  --pattern <square|circle>  Waypoint pattern (default: square)\n"
              << "  --size <val>               Square size or circle radius in meters (default: 5.0)\n"
              << "  --altitude <val>           Flight altitude in meters (default: 10.0)\n"
              << "  --speed <val>              Approach speed in m/s (default: 1.0)\n"
              << "  --points <val>             Number of points for circle (default: 8)\n"
              << "  --repeat                   Keep publishing periodically\n"
              << "  --interval <val>           Repeat interval in seconds (default: 30.0)\n"
              << std::endl;
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    // Parse arguments
    std::string pattern = "square";
    float size = 5.0f;
    float altitude = 10.0f;
    float speed = 1.0f;
    int points = 8;
    bool repeat = false;
    float interval = 30.0f;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage();
            return 0;
        }
        if (arg == "--pattern" && i + 1 < argc) pattern = argv[++i];
        else if (arg == "--size" && i + 1 < argc) size = std::stof(argv[++i]);
        else if (arg == "--altitude" && i + 1 < argc) altitude = std::stof(argv[++i]);
        else if (arg == "--speed" && i + 1 < argc) speed = std::stof(argv[++i]);
        else if (arg == "--points" && i + 1 < argc) points = std::stoi(argv[++i]);
        else if (arg == "--repeat") repeat = true;
        else if (arg == "--interval" && i + 1 < argc) interval = std::stof(argv[++i]);
    }

    std::cout << "=== Waypoint Publisher ===" << std::endl;
    std::cout << "Pattern: " << pattern << std::endl;
    std::cout << "Size: " << size << " m" << std::endl;
    std::cout << "Altitude: " << altitude << " m" << std::endl;
    std::cout << "Speed: " << speed << " m/s" << std::endl;
    if (pattern == "circle") {
        std::cout << "Points: " << points << std::endl;
    }
    std::cout << std::endl;

    // Initialize Zenoh session
    zenoh_interface::Session session;
    if (!session.is_valid()) {
        std::cerr << "Failed to create Zenoh session. Exiting." << std::endl;
        return 1;
    }

    const std::string waypoint_key = "robot/drone/cmd/waypoints";

    // Create waypoint list
    data_types::WaypointList waypoints;
    if (pattern == "circle") {
        waypoints = create_circle_pattern(size, altitude, speed, points);
    } else {
        waypoints = create_square_pattern(size, altitude, speed);
    }

    std::cout << "Publishing " << waypoints.size() << " waypoints to " << waypoint_key << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& wp = waypoints[i];
        std::cout << "  WP" << wp.id << ": (" << wp.x << ", " << wp.y << ", " << wp.z
                  << ") yaw=" << wp.yaw << " speed=" << wp.speed << std::endl;
    }
    std::cout << std::endl;

    // Publish waypoints
    auto payload = waypoints.serialize();
    if (session.publish(waypoint_key, payload)) {
        std::cout << "Waypoints published successfully (" << payload.size() << " bytes)" << std::endl;
    } else {
        std::cerr << "Failed to publish waypoints" << std::endl;
        return 1;
    }

    if (repeat) {
        std::cout << "Repeat mode: publishing every " << interval << " seconds" << std::endl;
        std::cout << "Press Ctrl+C to stop." << std::endl;

        while (true) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(interval * 1000)));

            // Update timestamp
            waypoints.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            payload = waypoints.serialize();
            if (session.publish(waypoint_key, payload)) {
                std::cout << "Waypoints republished" << std::endl;
            }
        }
    }

    return 0;
}
