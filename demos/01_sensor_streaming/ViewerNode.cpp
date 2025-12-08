// ViewerNode.cpp
// Subscribes to sensor data via Zenoh and displays it
// - Camera images displayed via OpenCV imshow (supports multiple cameras)
// - Odometry values printed to console
// - Uses wildcard subscription for multi-camera support

#include "ZenohInterface.hpp"
#include "ImageData.hpp"
#include "Odometry.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <csignal>
#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include <iomanip>

namespace {

std::atomic<bool> g_running{true};

// Per-camera state
struct CameraState {
    cv::Mat latest_image;
    bool new_image = false;
    int frame_count = 0;
};

std::mutex g_cameras_mutex;
std::map<std::string, CameraState> g_cameras;  // keyed by position (e.g., "front", "rgb")

std::mutex g_odometry_mutex;
data_types::Odometry g_latest_odometry;
bool g_new_odometry = false;

// Statistics
std::atomic<int> g_total_image_count{0};
std::atomic<int> g_odometry_count{0};

void signal_handler(int) {
    g_running = false;
}

// Extract camera position from key expression
// e.g., "robot/drone/sensor/camera/front" -> "front"
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
        cv::Mat mat = img_data.to_cv_mat();

        if (!mat.empty()) {
            std::string position = extract_camera_position(key);
            std::lock_guard<std::mutex> lock(g_cameras_mutex);
            auto& cam = g_cameras[position];
            cam.latest_image = mat.clone();
            cam.new_image = true;
            cam.frame_count++;
            g_total_image_count++;
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
              << "  --camera <key>   Subscribe to specific camera (default: robot/drone/sensor/camera/*)\n"
              << "  --no-odometry    Disable odometry subscription\n"
              << "  --grid           Display multiple cameras in a grid layout\n"
              << "  --help           Show this help\n";
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    // Parse arguments
    std::string camera_key = "robot/drone/sensor/camera/*";
    std::string odometry_key = "robot/drone/sensor/state/odom";
    bool subscribe_odometry = true;
    bool grid_mode = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--camera" && i + 1 < argc) {
            camera_key = argv[++i];
        } else if (arg == "--no-odometry") {
            subscribe_odometry = false;
        } else if (arg == "--grid") {
            grid_mode = true;
        } else if (arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
    }

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
    std::cout << "Press 'q' in any image window or Ctrl+C to quit." << std::endl;
    if (grid_mode) {
        std::cout << "Grid mode: Multiple cameras will be displayed in a single window." << std::endl;
    }

    auto start_time = std::chrono::steady_clock::now();
    auto last_stats_time = start_time;
    int last_image_count = 0;
    int last_odometry_count = 0;

    // Track which windows we've created
    std::set<std::string> created_windows;
    const std::string grid_window = "Drone Cameras (Grid)";

    while (g_running) {
        // Display latest images
        {
            std::lock_guard<std::mutex> lock(g_cameras_mutex);

            if (grid_mode && !g_cameras.empty()) {
                // Create grid layout
                std::vector<std::pair<std::string, cv::Mat>> valid_images;
                for (auto& [pos, cam] : g_cameras) {
                    if (!cam.latest_image.empty()) {
                        valid_images.emplace_back(pos, cam.latest_image);
                        cam.new_image = false;
                    }
                }

                if (!valid_images.empty()) {
                    // Calculate grid dimensions
                    int n = static_cast<int>(valid_images.size());
                    int cols = static_cast<int>(std::ceil(std::sqrt(n)));
                    int rows = static_cast<int>(std::ceil(static_cast<double>(n) / cols));

                    // Target cell size
                    int cell_w = 320;
                    int cell_h = 240;

                    cv::Mat grid(rows * cell_h, cols * cell_w, CV_8UC3, cv::Scalar(30, 30, 30));

                    for (size_t i = 0; i < valid_images.size(); ++i) {
                        int row = static_cast<int>(i) / cols;
                        int col = static_cast<int>(i) % cols;

                        cv::Mat resized;
                        cv::resize(valid_images[i].second, resized, cv::Size(cell_w, cell_h));

                        // Add label
                        cv::putText(resized, valid_images[i].first, cv::Point(5, 20),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

                        resized.copyTo(grid(cv::Rect(col * cell_w, row * cell_h, cell_w, cell_h)));
                    }

                    if (created_windows.find(grid_window) == created_windows.end()) {
                        cv::namedWindow(grid_window, cv::WINDOW_AUTOSIZE);
                        created_windows.insert(grid_window);
                    }
                    cv::imshow(grid_window, grid);
                }
            } else {
                // Individual windows per camera
                for (auto& [position, cam] : g_cameras) {
                    if (cam.new_image && !cam.latest_image.empty()) {
                        std::string window_name = "Camera: " + position;

                        if (created_windows.find(window_name) == created_windows.end()) {
                            cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
                            created_windows.insert(window_name);
                        }

                        cv::imshow(window_name, cam.latest_image);
                        cam.new_image = false;
                    }
                }
            }
        }

        // Print stats periodically
        auto now = std::chrono::steady_clock::now();
        if (now - last_stats_time >= std::chrono::seconds(1)) {
            int current_image_count = g_total_image_count.load();
            int current_odometry_count = g_odometry_count.load();

            int image_fps = current_image_count - last_image_count;
            int odometry_rate = current_odometry_count - last_odometry_count;

            std::cout << "[Stats] Total: " << image_fps << " fps";

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
                    std::cout << " Pos:(" << std::fixed << std::setprecision(1)
                              << g_latest_odometry.x << "," << g_latest_odometry.y << "," << g_latest_odometry.z << ")";
                }
            }
            std::cout << std::endl;

            last_image_count = current_image_count;
            last_odometry_count = current_odometry_count;
            last_stats_time = now;
        }

        // Handle OpenCV events and check for quit
        int key = cv::waitKey(10);
        if (key == 'q' || key == 'Q' || key == 27) {
            g_running = false;
        }
    }

    std::cout << std::endl;
    std::cout << "Shutting down..." << std::endl;
    std::cout << "Total images received: " << g_total_image_count.load() << std::endl;
    std::cout << "Total odometry messages: " << g_odometry_count.load() << std::endl;

    cv::destroyAllWindows();

    return 0;
}
