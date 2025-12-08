// ViewerNodeHeadless.cpp
// Headless version of ViewerNode for servers without display
// - Saves received images to files (optional)
// - Prints statistics and odometry to console

#include "ZenohInterface.hpp"
#include "ImageData.hpp"
#include "Odometry.hpp"

#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <csignal>
#include <filesystem>

namespace {

std::atomic<bool> g_running{true};

// Configuration
bool g_save_images = false;
std::string g_output_dir = "./captured_frames";
int g_save_interval = 30;  // Save every N frames

// Shared state for received data
std::mutex g_image_mutex;
data_types::ImageData g_latest_image;
bool g_new_image = false;

std::mutex g_odom_mutex;
data_types::Odometry g_latest_odom;
bool g_new_odom = false;

// Statistics
std::atomic<int> g_image_count{0};
std::atomic<int> g_odom_count{0};
std::atomic<size_t> g_total_image_bytes{0};

void signal_handler(int) {
    g_running = false;
}

void camera_callback(const std::string& key, const std::vector<uint8_t>& payload) {
    try {
        auto img_data = data_types::ImageData::deserialize(payload);

        if (img_data.is_valid()) {
            std::lock_guard<std::mutex> lock(g_image_mutex);
            g_latest_image = std::move(img_data);
            g_new_image = true;
            g_image_count++;
            g_total_image_bytes += payload.size();
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

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  --save-images         Save received images to disk\n"
              << "  --output-dir <path>   Directory for saved images (default: ./captured_frames)\n"
              << "  --save-interval <n>   Save every N frames (default: 30)\n"
              << "  --help                Show this help\n";
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--save-images") {
            g_save_images = true;
        } else if (arg == "--output-dir" && i + 1 < argc) {
            g_output_dir = argv[++i];
        } else if (arg == "--save-interval" && i + 1 < argc) {
            g_save_interval = std::stoi(argv[++i]);
        } else if (arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
    }

    // Setup signal handling
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=== Viewer Node (Headless) ===" << std::endl;

    if (g_save_images) {
        std::filesystem::create_directories(g_output_dir);
        std::cout << "Saving images to: " << g_output_dir << " (every " << g_save_interval << " frames)" << std::endl;
    }

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
    std::cout << "Press Ctrl+C to quit." << std::endl;
    std::cout << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    auto last_stats_time = start_time;
    int last_image_count = 0;
    int last_odom_count = 0;
    int saved_count = 0;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();

        // Save images periodically if enabled
        if (g_save_images) {
            std::lock_guard<std::mutex> lock(g_image_mutex);
            if (g_new_image && g_image_count % g_save_interval == 0) {
                cv::Mat mat = g_latest_image.to_cv_mat();
                if (!mat.empty()) {
                    std::string filename = g_output_dir + "/frame_" +
                                          std::to_string(g_image_count.load()) + ".png";
                    cv::imwrite(filename, mat);
                    saved_count++;
                }
            }
        }

        // Print stats every second
        if (now - last_stats_time >= std::chrono::seconds(1)) {
            int current_image_count = g_image_count.load();
            int current_odom_count = g_odom_count.load();
            size_t total_bytes = g_total_image_bytes.load();

            int image_fps = current_image_count - last_image_count;
            int odom_rate = current_odom_count - last_odom_count;
            double mbps = (image_fps * 921616.0 * 8) / 1000000.0;  // Approximate

            std::cout << "[Stats] ";
            std::cout << "Images: " << image_fps << " fps (" << std::fixed << std::setprecision(1) << mbps << " Mbps)";
            std::cout << " | Odom: " << odom_rate << " Hz";

            // Print latest odometry
            {
                std::lock_guard<std::mutex> lock(g_odom_mutex);
                if (g_odom_count > 0) {
                    std::cout << " | Pos: ("
                              << std::fixed << std::setprecision(2)
                              << g_latest_odom.x << ", "
                              << g_latest_odom.y << ", "
                              << g_latest_odom.z << ")";
                    std::cout << " Yaw: " << std::fixed << std::setprecision(2) << g_latest_odom.yaw;
                }
            }

            if (g_save_images) {
                std::cout << " | Saved: " << saved_count;
            }

            std::cout << std::endl;

            last_image_count = current_image_count;
            last_odom_count = current_odom_count;
            last_stats_time = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << std::endl;
    std::cout << "=== Summary ===" << std::endl;
    std::cout << "Total images received: " << g_image_count.load() << std::endl;
    std::cout << "Total odometry messages: " << g_odom_count.load() << std::endl;
    std::cout << "Total data received: " << (g_total_image_bytes.load() / 1024 / 1024) << " MB" << std::endl;
    if (g_save_images) {
        std::cout << "Images saved: " << saved_count << std::endl;
    }

    return 0;
}
