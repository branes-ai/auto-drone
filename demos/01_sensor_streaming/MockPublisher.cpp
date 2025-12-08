// MockPublisher.cpp
// Generates synthetic sensor data (camera images and odometry) and publishes via Zenoh
// This allows testing the Zenoh pipeline without requiring an actual simulator

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

namespace {

std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
}

// Generate a test pattern image with moving gradient
cv::Mat generate_test_image(int width, int height, int frame_number) {
    cv::Mat img(height, width, CV_8UC3);

    // Create a moving gradient pattern
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int offset = frame_number % 256;
            uint8_t r = static_cast<uint8_t>((x + offset) % 256);
            uint8_t g = static_cast<uint8_t>((y + offset) % 256);
            uint8_t b = static_cast<uint8_t>((x + y + offset) % 256);
            img.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);  // OpenCV uses BGR
        }
    }

    // Add frame number text
    cv::putText(img, "Frame: " + std::to_string(frame_number),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 255), 2);

    // Add a moving circle to make motion visible
    int cx = width / 2 + static_cast<int>(100 * std::cos(frame_number * 0.1));
    int cy = height / 2 + static_cast<int>(100 * std::sin(frame_number * 0.1));
    cv::circle(img, cv::Point(cx, cy), 20, cv::Scalar(0, 255, 0), -1);

    return img;
}

// Generate simulated odometry with circular motion
data_types::Odometry generate_odometry(double time_sec) {
    // Simulate a drone flying in a circle
    float radius = 5.0f;  // meters
    float angular_velocity = 0.2f;  // rad/s
    float altitude = 10.0f;  // meters

    float angle = static_cast<float>(time_sec) * angular_velocity;

    data_types::Odometry odom;
    odom.x = radius * std::cos(angle);
    odom.y = radius * std::sin(angle);
    odom.z = altitude + 0.5f * std::sin(angle * 2);  // Slight altitude variation
    odom.roll = 0.1f * std::sin(angle);  // Slight roll during turns
    odom.pitch = 0.05f * std::cos(angle);
    odom.yaw = angle;  // Heading follows the circle

    auto now = std::chrono::system_clock::now();
    odom.timestamp_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count());

    return odom;
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    // Setup signal handling for graceful shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Parse arguments
    int image_width = 640;
    int image_height = 480;
    int image_fps = 30;
    int odom_rate = 100;  // Hz

    std::cout << "=== Mock Publisher ===" << std::endl;
    std::cout << "Image: " << image_width << "x" << image_height << " @ " << image_fps << " FPS" << std::endl;
    std::cout << "Odometry: " << odom_rate << " Hz" << std::endl;
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

    const auto image_interval = std::chrono::microseconds(1000000 / image_fps);
    const auto odom_interval = std::chrono::microseconds(1000000 / odom_rate);

    int frame_count = 0;
    int odom_count = 0;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - start_time).count();

        // Publish image at specified FPS
        if (now - last_image_time >= image_interval) {
            cv::Mat img = generate_test_image(image_width, image_height, frame_count);
            auto img_data = data_types::ImageData::from_cv_mat(img);
            auto payload = img_data.serialize();

            if (session.publish(camera_key, payload)) {
                frame_count++;
                if (frame_count % image_fps == 0) {
                    std::cout << "[Camera] Published frame " << frame_count
                              << " (" << payload.size() << " bytes)" << std::endl;
                }
            }
            last_image_time = now;
        }

        // Publish odometry at specified rate
        if (now - last_odom_time >= odom_interval) {
            auto odom = generate_odometry(elapsed);
            auto payload = odom.serialize();

            if (session.publish(odom_key, payload)) {
                odom_count++;
                if (odom_count % odom_rate == 0) {
                    std::cout << "[Odometry] x=" << odom.x << " y=" << odom.y << " z=" << odom.z
                              << " yaw=" << odom.yaw << std::endl;
                }
            }
            last_odom_time = now;
        }

        // Sleep briefly to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    std::cout << std::endl;
    std::cout << "Shutting down..." << std::endl;
    std::cout << "Total frames published: " << frame_count << std::endl;
    std::cout << "Total odometry messages: " << odom_count << std::endl;

    return 0;
}
