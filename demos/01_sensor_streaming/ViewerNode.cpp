// ViewerNode.cpp
// Subscribes to sensor data via Zenoh and displays it
// - Camera images displayed via OpenCV imshow
// - Odometry values printed to console

#include "ZenohInterface.hpp"
#include "ImageData.hpp"
#include "Odometry.hpp"

#include <opencv2/highgui.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <csignal>

namespace {

std::atomic<bool> g_running{true};

// Shared state for received data
std::mutex g_image_mutex;
cv::Mat g_latest_image;
bool g_new_image = false;

std::mutex g_odom_mutex;
data_types::Odometry g_latest_odom;
bool g_new_odom = false;

// Statistics
std::atomic<int> g_image_count{0};
std::atomic<int> g_odom_count{0};

void signal_handler(int) {
    g_running = false;
}

void camera_callback(const std::string& key, const std::vector<uint8_t>& payload) {
    try {
        auto img_data = data_types::ImageData::deserialize(payload);
        cv::Mat mat = img_data.to_cv_mat();

        if (!mat.empty()) {
            std::lock_guard<std::mutex> lock(g_image_mutex);
            g_latest_image = mat.clone();
            g_new_image = true;
            g_image_count++;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error deserializing image: " << e.what() << std::endl;
    }
}

void odom_callback(const std::string& key, const std::vector<uint8_t>& payload) {
    try {
        auto odom = data_types::Odometry::deserialize(payload);

        {
            std::lock_guard<std::mutex> lock(g_odom_mutex);
            g_latest_odom = odom;
            g_new_odom = true;
        }
        g_odom_count++;
    } catch (const std::exception& e) {
        std::cerr << "Error deserializing odometry: " << e.what() << std::endl;
    }
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    // Setup signal handling
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=== Viewer Node ===" << std::endl;

    // Initialize Zenoh session
    zenoh_interface::Session session;
    if (!session.is_valid()) {
        std::cerr << "Failed to create Zenoh session. Exiting." << std::endl;
        return 1;
    }

    // Zenoh key expressions
    const std::string camera_key = "robot/drone/sensor/camera/rgb";
    const std::string odom_key = "robot/drone/sensor/state/odom";

    // Subscribe to camera feed
    int camera_sub = session.subscribe(camera_key, camera_callback);
    if (camera_sub < 0) {
        std::cerr << "Failed to subscribe to camera feed." << std::endl;
        return 1;
    }

    // Subscribe to odometry
    int odom_sub = session.subscribe(odom_key, odom_callback);
    if (odom_sub < 0) {
        std::cerr << "Failed to subscribe to odometry." << std::endl;
        return 1;
    }

    std::cout << "Subscribed to:" << std::endl;
    std::cout << "  Camera: " << camera_key << std::endl;
    std::cout << "  Odometry: " << odom_key << std::endl;
    std::cout << std::endl;
    std::cout << "Press 'q' in the image window or Ctrl+C to quit." << std::endl;

    // Create window
    const std::string window_name = "Drone Camera Feed";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    auto start_time = std::chrono::steady_clock::now();
    auto last_stats_time = start_time;
    int last_image_count = 0;
    int last_odom_count = 0;

    while (g_running) {
        // Display latest image if available
        {
            std::lock_guard<std::mutex> lock(g_image_mutex);
            if (g_new_image && !g_latest_image.empty()) {
                cv::imshow(window_name, g_latest_image);
                g_new_image = false;
            }
        }

        // Print odometry periodically
        auto now = std::chrono::steady_clock::now();
        if (now - last_stats_time >= std::chrono::seconds(1)) {
            int current_image_count = g_image_count.load();
            int current_odom_count = g_odom_count.load();

            int image_fps = current_image_count - last_image_count;
            int odom_rate = current_odom_count - last_odom_count;

            // Print latest odometry
            {
                std::lock_guard<std::mutex> lock(g_odom_mutex);
                if (g_new_odom) {
                    std::cout << "[Stats] Image: " << image_fps << " fps, Odom: " << odom_rate << " Hz"
                              << " | Pos: (" << g_latest_odom.x << ", " << g_latest_odom.y << ", " << g_latest_odom.z << ")"
                              << " | Yaw: " << g_latest_odom.yaw << std::endl;
                    g_new_odom = false;
                } else {
                    std::cout << "[Stats] Image: " << image_fps << " fps, Odom: " << odom_rate << " Hz"
                              << " | Waiting for data..." << std::endl;
                }
            }

            last_image_count = current_image_count;
            last_odom_count = current_odom_count;
            last_stats_time = now;
        }

        // Handle OpenCV events and check for quit
        int key = cv::waitKey(10);
        if (key == 'q' || key == 'Q' || key == 27) {  // q, Q, or ESC
            g_running = false;
        }
    }

    std::cout << std::endl;
    std::cout << "Shutting down..." << std::endl;
    std::cout << "Total images received: " << g_image_count.load() << std::endl;
    std::cout << "Total odometry messages: " << g_odom_count.load() << std::endl;

    cv::destroyAllWindows();

    return 0;
}
