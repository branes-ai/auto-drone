// MockTargetPublisher.cpp
// Generates synthetic camera images with a moving colored target for testing
// the object tracking pipeline without a real camera or simulator.

#include "ZenohInterface.hpp"
#include "ImageData.hpp"
#include "Odometry.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <csignal>
#include <atomic>
#include <string>

namespace {

std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
}

// Configuration
struct Config {
    int image_width = 640;
    int image_height = 480;
    int image_fps = 30;
    int odom_rate = 100;

    // Target appearance
    int target_radius = 30;          // pixels
    cv::Scalar target_color{0, 0, 255};  // Red in BGR

    // Target motion (circular pattern)
    float motion_radius = 150.0f;    // pixels from center
    float motion_speed = 0.5f;       // radians per second
    bool enable_motion = true;
};

void print_usage() {
    std::cout << "Usage: mock_target_publisher [options]\n"
              << "Options:\n"
              << "  --width <val>       Image width (default: 640)\n"
              << "  --height <val>      Image height (default: 480)\n"
              << "  --fps <val>         Image FPS (default: 30)\n"
              << "  --radius <val>      Target radius pixels (default: 30)\n"
              << "  --motion-radius <val> Motion radius pixels (default: 150)\n"
              << "  --motion-speed <val>  Motion speed rad/s (default: 0.5)\n"
              << "  --static            Disable target motion\n"
              << std::endl;
}

void parse_args(int argc, char* argv[], Config& config) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage();
            std::exit(0);
        }
        if (arg == "--width" && i + 1 < argc) config.image_width = std::stoi(argv[++i]);
        else if (arg == "--height" && i + 1 < argc) config.image_height = std::stoi(argv[++i]);
        else if (arg == "--fps" && i + 1 < argc) config.image_fps = std::stoi(argv[++i]);
        else if (arg == "--radius" && i + 1 < argc) config.target_radius = std::stoi(argv[++i]);
        else if (arg == "--motion-radius" && i + 1 < argc) config.motion_radius = std::stof(argv[++i]);
        else if (arg == "--motion-speed" && i + 1 < argc) config.motion_speed = std::stof(argv[++i]);
        else if (arg == "--static") config.enable_motion = false;
    }
}

// Generate an image with a colored target
cv::Mat generate_target_image(const Config& config, double elapsed_time, int frame_number) {
    cv::Mat img(config.image_height, config.image_width, CV_8UC3);

    // Gray background with some texture
    for (int y = 0; y < config.image_height; ++y) {
        for (int x = 0; x < config.image_width; ++x) {
            uint8_t gray = 100 + (x % 20 < 10 ? 10 : 0) + (y % 20 < 10 ? 10 : 0);
            img.at<cv::Vec3b>(y, x) = cv::Vec3b(gray, gray, gray);
        }
    }

    // Calculate target position
    float cx = config.image_width / 2.0f;
    float cy = config.image_height / 2.0f;

    if (config.enable_motion) {
        float angle = static_cast<float>(elapsed_time) * config.motion_speed;
        cx += config.motion_radius * std::cos(angle);
        cy += config.motion_radius * std::sin(angle);
    }

    // Draw target (filled circle)
    cv::circle(img, cv::Point(static_cast<int>(cx), static_cast<int>(cy)),
               config.target_radius, config.target_color, -1);

    // Add a white border for visibility
    cv::circle(img, cv::Point(static_cast<int>(cx), static_cast<int>(cy)),
               config.target_radius + 2, cv::Scalar(255, 255, 255), 2);

    // Add frame info text
    cv::putText(img, "Frame: " + std::to_string(frame_number),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 1);

    cv::putText(img, "Target: (" + std::to_string(static_cast<int>(cx)) + ", " +
                std::to_string(static_cast<int>(cy)) + ")",
                cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 1);

    return img;
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    // Setup signal handling
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    Config config;
    parse_args(argc, argv, config);

    std::cout << "=== Mock Target Publisher ===" << std::endl;
    std::cout << "Image: " << config.image_width << "x" << config.image_height
              << " @ " << config.image_fps << " FPS" << std::endl;
    std::cout << "Target radius: " << config.target_radius << " pixels" << std::endl;
    if (config.enable_motion) {
        std::cout << "Motion: circular, radius=" << config.motion_radius
                  << " speed=" << config.motion_speed << " rad/s" << std::endl;
    } else {
        std::cout << "Motion: static (center)" << std::endl;
    }
    std::cout << std::endl;

    // Initialize Zenoh session
    zenoh_interface::Session session;
    if (!session.is_valid()) {
        std::cerr << "Failed to create Zenoh session. Exiting." << std::endl;
        return 1;
    }

    // Zenoh key expressions
    const std::string camera_key = "robot/drone/sensor/camera/rgb";
    const std::string odom_key = "robot/drone/sensor/state/odom";

    std::cout << "Publishing to:" << std::endl;
    std::cout << "  Camera: " << camera_key << std::endl;
    std::cout << "  Odometry: " << odom_key << std::endl;
    std::cout << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    // Timing
    auto start_time = std::chrono::steady_clock::now();
    auto last_image_time = start_time;
    auto last_odom_time = start_time;

    const auto image_interval = std::chrono::microseconds(1000000 / config.image_fps);
    const auto odom_interval = std::chrono::microseconds(1000000 / config.odom_rate);

    int frame_count = 0;
    int odom_count = 0;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();

        // Publish image
        if (now - last_image_time >= image_interval) {
            cv::Mat img = generate_target_image(config, elapsed, frame_count);
            auto img_data = data_types::ImageData::from_cv_mat(img);
            auto payload = img_data.serialize();

            if (session.publish(camera_key, payload)) {
                frame_count++;
                if (frame_count % config.image_fps == 0) {
                    std::cout << "[Camera] Published frame " << frame_count
                              << " (" << payload.size() << " bytes)" << std::endl;
                }
            }
            last_image_time = now;
        }

        // Publish odometry (stationary drone for simplicity)
        if (now - last_odom_time >= odom_interval) {
            auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            data_types::Odometry odom(0.0f, 0.0f, -10.0f, 0.0f, 0.0f, 0.0f, now_us);

            if (session.publish(odom_key, odom.serialize())) {
                odom_count++;
            }
            last_odom_time = now;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    std::cout << std::endl;
    std::cout << "Shutting down..." << std::endl;
    std::cout << "Total frames: " << frame_count << std::endl;
    std::cout << "Total odometry: " << odom_count << std::endl;

    return 0;
}
