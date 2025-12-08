// ViewerNodeHeadless.cpp
// Headless version of ViewerNode for servers without display
// - Saves received images to files (optional)
// - Prints statistics and odometry to console
// - Supports multiple cameras via wildcard subscription

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
#include <map>
#include <iomanip>

namespace {

std::atomic<bool> g_running{true};

// Configuration
bool g_save_images = false;
std::string g_output_dir = "./captured_frames";
int g_save_interval = 30;  // Save every N frames

// Per-camera state
struct CameraState {
    data_types::ImageData latest_image;
    bool new_image = false;
    int frame_count = 0;
    size_t total_bytes = 0;
};

std::mutex g_cameras_mutex;
std::map<std::string, CameraState> g_cameras;

std::mutex g_odometry_mutex;
data_types::Odometry g_latest_odometry;
bool g_new_odometry = false;

// Statistics
std::atomic<int> g_total_image_count{0};
std::atomic<int> g_odometry_count{0};
std::atomic<size_t> g_total_image_bytes{0};

void signal_handler(int) {
    g_running = false;
}

// Extract camera position from key expression
std::string extract_camera_position(const std::string& key) {
    size_t last_slash = key.rfind('/');
    if (last_slash != std::string::npos) {
        return key.substr(last_slash + 1);
    }
    return key;
}

void camera_callback(const std::string& key, const std::vector<uint8_t>& payload) {
    try {
        auto img_data = data_types::ImageData::deserialize(payload);

        if (img_data.is_valid()) {
            std::string position = extract_camera_position(key);
            std::lock_guard<std::mutex> lock(g_cameras_mutex);
            auto& cam = g_cameras[position];
            cam.latest_image = std::move(img_data);
            cam.new_image = true;
            cam.frame_count++;
            cam.total_bytes += payload.size();
            g_total_image_count++;
            g_total_image_bytes += payload.size();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error deserializing image: " << e.what() << std::endl;
    }
}

void odometry_callback(const std::string& key, const std::vector<uint8_t>& payload) {
    try {
        auto odometry = data_types::Odometry::deserialize(payload);

        {
            std::lock_guard<std::mutex> lock(g_odometry_mutex);
            g_latest_odometry = odometry;
            g_new_odometry = true;
        }
        g_odometry_count++;
    } catch (const std::exception& e) {
        std::cerr << "Error deserializing odometry: " << e.what() << std::endl;
    }
}

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  --camera <key>        Camera subscription key (default: robot/drone/sensor/camera/*)\n"
              << "  --save-images         Save received images to disk\n"
              << "  --output-dir <path>   Directory for saved images (default: ./captured_frames)\n"
              << "  --save-interval <n>   Save every N frames per camera (default: 30)\n"
              << "  --no-odometry         Disable odometry subscription\n"
              << "  --help                Show this help\n";
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    // Parse arguments
    std::string camera_key = "robot/drone/sensor/camera/*";
    std::string odometry_key = "robot/drone/sensor/state/odom";
    bool subscribe_odometry = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--camera" && i + 1 < argc) {
            camera_key = argv[++i];
        } else if (arg == "--save-images") {
            g_save_images = true;
        } else if (arg == "--output-dir" && i + 1 < argc) {
            g_output_dir = argv[++i];
        } else if (arg == "--save-interval" && i + 1 < argc) {
            g_save_interval = std::stoi(argv[++i]);
        } else if (arg == "--no-odometry") {
            subscribe_odometry = false;
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
        std::cout << "Saving images to: " << g_output_dir << " (every " << g_save_interval << " frames per camera)" << std::endl;
    }

    // Initialize Zenoh session
    zenoh_interface::Session session;
    if (!session.is_valid()) {
        std::cerr << "Failed to create Zenoh session. Exiting." << std::endl;
        return 1;
    }

    // Subscribe to camera feed (wildcard for multi-camera support)
    int camera_sub = session.subscribe(camera_key, camera_callback);
    if (camera_sub < 0) {
        std::cerr << "Failed to subscribe to camera feed." << std::endl;
        return 1;
    }

    std::cout << "Subscribed to:" << std::endl;
    std::cout << "  Camera: " << camera_key << std::endl;

    // Subscribe to odometry
    int odometry_sub = -1;
    if (subscribe_odometry) {
        odometry_sub = session.subscribe(odometry_key, odometry_callback);
        if (odometry_sub < 0) {
            std::cerr << "Warning: Failed to subscribe to odometry." << std::endl;
        } else {
            std::cout << "  Odometry: " << odometry_key << std::endl;
        }
    }

    std::cout << std::endl;
    std::cout << "Press Ctrl+C to quit." << std::endl;
    std::cout << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    auto last_stats_time = start_time;
    int last_image_count = 0;
    int last_odometry_count = 0;
    std::map<std::string, int> saved_counts;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();

        // Save images periodically if enabled
        if (g_save_images) {
            std::lock_guard<std::mutex> lock(g_cameras_mutex);
            for (auto& [position, cam] : g_cameras) {
                if (cam.new_image && cam.frame_count % g_save_interval == 0) {
                    cv::Mat mat = cam.latest_image.to_cv_mat();
                    if (!mat.empty()) {
                        // Create per-camera subdirectory
                        std::string cam_dir = g_output_dir + "/" + position;
                        std::filesystem::create_directories(cam_dir);

                        std::string filename = cam_dir + "/frame_" +
                                              std::to_string(cam.frame_count) + ".png";
                        cv::imwrite(filename, mat);
                        saved_counts[position]++;
                    }
                    cam.new_image = false;
                }
            }
        }

        // Print stats every second
        if (now - last_stats_time >= std::chrono::seconds(1)) {
            int current_image_count = g_total_image_count.load();
            int current_odometry_count = g_odometry_count.load();
            size_t total_bytes = g_total_image_bytes.load();

            int image_fps = current_image_count - last_image_count;
            int odometry_rate = current_odometry_count - last_odometry_count;

            // Calculate actual bandwidth
            double elapsed_sec = std::chrono::duration<double>(now - last_stats_time).count();
            double mbps = 0;
            {
                std::lock_guard<std::mutex> lock(g_cameras_mutex);
                size_t bytes_this_sec = 0;
                for (const auto& [pos, cam] : g_cameras) {
                    bytes_this_sec += cam.total_bytes;
                }
                // This is cumulative, so we estimate from frame count and average size
                if (current_image_count > 0) {
                    double avg_bytes = static_cast<double>(total_bytes) / current_image_count;
                    mbps = (image_fps * avg_bytes * 8) / 1000000.0;
                }
            }

            std::cout << "[Stats] Total: " << image_fps << " fps (" << std::fixed << std::setprecision(1) << mbps << " Mbps)";

            // Per-camera stats
            {
                std::lock_guard<std::mutex> lock(g_cameras_mutex);
                for (const auto& [pos, cam] : g_cameras) {
                    std::cout << " | " << pos << ":" << cam.frame_count;
                }
            }

            if (subscribe_odometry) {
                std::cout << " | Odometry: " << odometry_rate << " Hz";
                std::lock_guard<std::mutex> lock(g_odometry_mutex);
                if (g_odometry_count > 0) {
                    std::cout << " Pos:("
                              << std::fixed << std::setprecision(1)
                              << g_latest_odometry.x << ","
                              << g_latest_odometry.y << ","
                              << g_latest_odometry.z << ")";
                }
            }

            if (g_save_images) {
                int total_saved = 0;
                for (const auto& [pos, count] : saved_counts) {
                    total_saved += count;
                }
                std::cout << " | Saved: " << total_saved;
            }

            std::cout << std::endl;

            last_image_count = current_image_count;
            last_odometry_count = current_odometry_count;
            last_stats_time = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << std::endl;
    std::cout << "=== Summary ===" << std::endl;
    std::cout << "Total images received: " << g_total_image_count.load() << std::endl;
    {
        std::lock_guard<std::mutex> lock(g_cameras_mutex);
        for (const auto& [pos, cam] : g_cameras) {
            std::cout << "  " << pos << ": " << cam.frame_count << " frames" << std::endl;
        }
    }
    std::cout << "Total odometry messages: " << g_odometry_count.load() << std::endl;
    std::cout << "Total data received: " << (g_total_image_bytes.load() / 1024 / 1024) << " MB" << std::endl;
    if (g_save_images) {
        std::cout << "Images saved per camera:" << std::endl;
        for (const auto& [pos, count] : saved_counts) {
            std::cout << "  " << pos << ": " << count << std::endl;
        }
    }

    return 0;
}
