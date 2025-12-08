// WebcamPublisher.cpp
// Captures from physical webcams and publishes via Zenoh
// Supports multiple cameras with configurable positions (front, back, left, right, top, bottom)

#include "ZenohInterface.hpp"
#include "ImageData.hpp"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <vector>
#include <string>
#include <map>
#include <getopt.h>

namespace {

std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
}

// Camera position names for Zenoh key expressions
// Designed for Skydio-style 6-camera configuration
const std::vector<std::string> CAMERA_POSITIONS = {
    "front",   // Camera 0
    "back",    // Camera 1
    "left",    // Camera 2
    "right",   // Camera 3
    "top",     // Camera 4
    "bottom"   // Camera 5
};

struct CameraInfo {
    int index;
    std::string position;
    cv::VideoCapture capture;
    int width;
    int height;
    double fps;
    bool active;
};

// Probe available cameras and return list of valid capture device indices
// On Linux, cameras often expose multiple /dev/videoN nodes (capture + metadata)
// We need to find only the actual capture devices
std::vector<int> probe_cameras(int max_probe = 16) {
    std::vector<int> valid_indices;

    for (int i = 0; i < max_probe; ++i) {
        cv::VideoCapture cap;

        // Try to open with V4L2 backend explicitly on Linux
        #ifdef __linux__
        cap.open(i, cv::CAP_V4L2);
        #else
        cap.open(i);
        #endif

        if (!cap.isOpened()) {
            continue;  // Don't stop - there may be gaps (e.g., video0, video2)
        }

        // Try to grab a frame to verify it's a real capture device
        cv::Mat test_frame;
        bool can_capture = cap.read(test_frame);
        cap.release();

        if (can_capture && !test_frame.empty()) {
            valid_indices.push_back(i);
            std::cout << "  Found capture device at index " << i << std::endl;
        }
    }

    return valid_indices;
}

// Initialize cameras with native resolution
// Takes a list of valid device indices from probe_cameras()
std::vector<CameraInfo> init_cameras(const std::vector<int>& device_indices, int target_fps) {
    std::vector<CameraInfo> cameras;

    for (size_t i = 0; i < device_indices.size() && i < CAMERA_POSITIONS.size(); ++i) {
        int dev_index = device_indices[i];
        CameraInfo cam;
        cam.index = dev_index;
        cam.position = CAMERA_POSITIONS[i];

        #ifdef __linux__
        cam.capture.open(dev_index, cv::CAP_V4L2);
        #else
        cam.capture.open(dev_index);
        #endif

        if (!cam.capture.isOpened()) {
            std::cerr << "Warning: Could not open camera at index " << dev_index << std::endl;
            cam.active = false;
            cameras.push_back(std::move(cam));
            continue;
        }

        // Request MJPEG format to reduce USB bandwidth
        // This is critical for multi-camera setups on USB 2.0
        // YUYV 640x480@30fps = ~18MB/s per camera
        // MJPEG 640x480@30fps = ~2MB/s per camera
        cam.capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        // Set target FPS (camera may not support it exactly)
        cam.capture.set(cv::CAP_PROP_FPS, target_fps);

        // Get actual camera properties (native resolution)
        cam.width = static_cast<int>(cam.capture.get(cv::CAP_PROP_FRAME_WIDTH));
        cam.height = static_cast<int>(cam.capture.get(cv::CAP_PROP_FRAME_HEIGHT));
        cam.fps = cam.capture.get(cv::CAP_PROP_FPS);
        cam.active = true;

        std::cout << "Camera " << dev_index << " (" << cam.position << "): "
                  << cam.width << "x" << cam.height << " @ " << cam.fps << " FPS" << std::endl;

        cameras.push_back(std::move(cam));
    }

    return cameras;
}

void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [OPTIONS]\n"
              << "\n"
              << "Options:\n"
              << "  -n, --num-cameras N   Number of cameras to use (default: auto-detect)\n"
              << "  -f, --fps N           Target frame rate (default: 30)\n"
              << "  -h, --help            Show this help message\n"
              << "\n"
              << "Camera positions (Skydio-style):\n"
              << "  Camera 0: front\n"
              << "  Camera 1: back\n"
              << "  Camera 2: left\n"
              << "  Camera 3: right\n"
              << "  Camera 4: top\n"
              << "  Camera 5: bottom\n"
              << "\n"
              << "Zenoh topics:\n"
              << "  robot/drone/sensor/camera/{position}  (e.g., robot/drone/sensor/camera/front)\n"
              << std::endl;
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    // Setup signal handling for graceful shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Default options
    int num_cameras = -1;  // -1 means auto-detect
    int target_fps = 30;

    // Parse command line arguments
    static struct option long_options[] = {
        {"num-cameras", required_argument, nullptr, 'n'},
        {"fps",         required_argument, nullptr, 'f'},
        {"help",        no_argument,       nullptr, 'h'},
        {nullptr,       0,                 nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "n:f:h", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'n':
                num_cameras = std::atoi(optarg);
                break;
            case 'f':
                target_fps = std::atoi(optarg);
                break;
            case 'h':
                print_usage(argv[0]);
                return 0;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    std::cout << "=== Webcam Publisher ===" << std::endl;
    std::cout << std::endl;

    // Probe for available cameras
    std::cout << "Probing for capture devices..." << std::endl;
    auto device_indices = probe_cameras();

    if (device_indices.empty()) {
        std::cerr << "No capture devices detected. Exiting." << std::endl;
        return 1;
    }
    std::cout << "Detected " << device_indices.size() << " capture device(s)" << std::endl;

    // Limit to requested number of cameras if specified
    if (num_cameras > 0 && static_cast<size_t>(num_cameras) < device_indices.size()) {
        device_indices.resize(num_cameras);
    }

    // Cap at 6 cameras (Skydio-style maximum)
    if (device_indices.size() > 6) {
        std::cout << "Limiting to 6 cameras (Skydio-style configuration)" << std::endl;
        device_indices.resize(6);
    }

    std::cout << std::endl;

    // Initialize cameras
    auto cameras = init_cameras(device_indices, target_fps);

    // Count active cameras
    int active_count = 0;
    for (const auto& cam : cameras) {
        if (cam.active) active_count++;
    }

    if (active_count == 0) {
        std::cerr << "No cameras could be opened. Exiting." << std::endl;
        return 1;
    }

    std::cout << std::endl;

    // Initialize Zenoh session
    zenoh_interface::Session session;
    if (!session.is_valid()) {
        std::cerr << "Failed to create Zenoh session. Exiting." << std::endl;
        return 1;
    }

    // Build key expressions for each camera
    std::map<int, std::string> camera_keys;
    std::cout << "Publishing to:" << std::endl;
    for (const auto& cam : cameras) {
        if (cam.active) {
            std::string key = "robot/drone/sensor/camera/" + cam.position;
            camera_keys[cam.index] = key;
            std::cout << "  Camera " << cam.index << ": " << key << std::endl;
        }
    }

    std::cout << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;
    std::cout << std::endl;

    // Timing
    auto start_time = std::chrono::steady_clock::now();
    auto last_frame_time = start_time;
    const auto frame_interval = std::chrono::microseconds(1000000 / target_fps);

    // Frame counters per camera
    std::vector<int> frame_counts(cameras.size(), 0);
    int total_frames = 0;

    // Pre-allocate frame storage
    std::vector<cv::Mat> frames(cameras.size());

    while (g_running) {
        auto now = std::chrono::steady_clock::now();

        // Capture and publish at target FPS
        if (now - last_frame_time >= frame_interval) {
            // Capture from all cameras (minimize time skew between cameras)
            for (size_t i = 0; i < cameras.size(); ++i) {
                if (cameras[i].active) {
                    cameras[i].capture >> frames[i];
                }
            }

            // Publish all frames
            for (size_t i = 0; i < cameras.size(); ++i) {
                if (!cameras[i].active || frames[i].empty()) continue;

                auto img_data = data_types::ImageData::from_cv_mat(frames[i]);
                auto payload = img_data.serialize();

                if (session.publish(camera_keys[cameras[i].index], payload)) {
                    frame_counts[i]++;
                    total_frames++;
                }
            }

            // Status output every second
            if (total_frames > 0 && total_frames % (target_fps * active_count) == 0) {
                auto elapsed = std::chrono::duration<double>(now - start_time).count();
                std::cout << "[" << static_cast<int>(elapsed) << "s] ";
                for (size_t i = 0; i < cameras.size(); ++i) {
                    if (cameras[i].active) {
                        std::cout << cameras[i].position << ":" << frame_counts[i] << " ";
                    }
                }
                std::cout << std::endl;
            }

            last_frame_time = now;
        }

        // Sleep briefly to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    std::cout << std::endl;
    std::cout << "Shutting down..." << std::endl;
    std::cout << "Frames published per camera:" << std::endl;
    for (size_t i = 0; i < cameras.size(); ++i) {
        if (cameras[i].active) {
            std::cout << "  " << cameras[i].position << ": " << frame_counts[i] << std::endl;
        }
    }
    std::cout << "Total frames: " << total_frames << std::endl;

    // Release cameras
    for (auto& cam : cameras) {
        if (cam.capture.isOpened()) {
            cam.capture.release();
        }
    }

    return 0;
}
