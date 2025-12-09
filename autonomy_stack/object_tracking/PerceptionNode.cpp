// PerceptionNode.cpp
// Subscribes to camera images, runs object detection, and publishes object detections
// with estimated 3D world positions.

#include "PerceptionEngine.hpp"
#include "ZenohInterface.hpp"
#include "ImageData.hpp"
#include "ObjectDetection.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <mutex>
#include <string>

namespace {

std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
}

// Configuration
struct Config {
    // Camera intrinsics
    int image_width = 640;
    int image_height = 480;
    float hfov_deg = 70.0f;

    // Detection parameters
    int h_min = 0;      // HSV hue min (red)
    int h_max = 10;     // HSV hue max
    int s_min = 100;    // Saturation min
    int v_min = 100;    // Value min
    int min_area = 100; // Minimum blob area

    // 3D estimation
    float object_width = 0.3f;  // Assumed object width in meters
};

void print_usage() {
    std::cout << "Usage: perception_node [options]\n"
              << "Options:\n"
              << "  --width <val>       Image width (default: 640)\n"
              << "  --height <val>      Image height (default: 480)\n"
              << "  --hfov <val>        Horizontal FOV in degrees (default: 70)\n"
              << "  --h-min <val>       HSV hue min (default: 0)\n"
              << "  --h-max <val>       HSV hue max (default: 10)\n"
              << "  --s-min <val>       HSV saturation min (default: 100)\n"
              << "  --v-min <val>       HSV value min (default: 100)\n"
              << "  --min-area <val>    Minimum blob area (default: 100)\n"
              << "  --object-width <val> Object width for depth estimation (default: 0.3)\n"
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
        else if (arg == "--hfov" && i + 1 < argc) config.hfov_deg = std::stof(argv[++i]);
        else if (arg == "--h-min" && i + 1 < argc) config.h_min = std::stoi(argv[++i]);
        else if (arg == "--h-max" && i + 1 < argc) config.h_max = std::stoi(argv[++i]);
        else if (arg == "--s-min" && i + 1 < argc) config.s_min = std::stoi(argv[++i]);
        else if (arg == "--v-min" && i + 1 < argc) config.v_min = std::stoi(argv[++i]);
        else if (arg == "--min-area" && i + 1 < argc) config.min_area = std::stoi(argv[++i]);
        else if (arg == "--object-width" && i + 1 < argc) config.object_width = std::stof(argv[++i]);
    }
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    // Setup signal handling
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    Config config;
    parse_args(argc, argv, config);

    std::cout << "=== Perception Node ===" << std::endl;
    std::cout << "Image: " << config.image_width << "x" << config.image_height << std::endl;
    std::cout << "HFOV: " << config.hfov_deg << " degrees" << std::endl;
    std::cout << "HSV range: H[" << config.h_min << "-" << config.h_max << "] S>="
              << config.s_min << " V>=" << config.v_min << std::endl;
    std::cout << "Min area: " << config.min_area << " pixels" << std::endl;
    std::cout << "Object width: " << config.object_width << " m" << std::endl;
    std::cout << std::endl;

    // Initialize Zenoh session
    zenoh_interface::Session session;
    if (!session.is_valid()) {
        std::cerr << "Failed to create Zenoh session. Exiting." << std::endl;
        return 1;
    }

    // Setup perception engine
    autonomy_stack::PerceptionEngine::Config engine_config;
    engine_config.camera = autonomy_stack::CameraIntrinsics::from_resolution(
        config.image_width, config.image_height, config.hfov_deg);
    engine_config.reference_object_width = config.object_width;

    autonomy_stack::PerceptionEngine engine(engine_config);

    // Setup color detector
    autonomy_stack::ColorBlobDetector::Config detector_config;
    detector_config.h_min = config.h_min;
    detector_config.h_max = config.h_max;
    detector_config.s_min = config.s_min;
    detector_config.v_min = config.v_min;
    detector_config.min_area = config.min_area;
    engine.set_detector(std::make_unique<autonomy_stack::ColorBlobDetector>(detector_config));

    // Zenoh key expressions
    const std::string camera_key = "robot/drone/sensor/camera/rgb";
    const std::string detection_key = "robot/drone/perception/objects";

    // State
    std::mutex state_mutex;
    uint32_t frame_count = 0;
    uint32_t detection_count = 0;

    // Subscribe to camera images
    session.subscribe(camera_key, [&](const std::string&, const std::vector<uint8_t>& payload) {
        try {
            auto image_data = data_types::ImageData::deserialize(payload);
            cv::Mat image = image_data.to_cv_mat();

            if (image.empty()) {
                return;
            }

            uint32_t current_frame;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                current_frame = frame_count++;
            }

            // Run perception
            auto detections = engine.process(image, current_frame);

            // Publish detections
            if (session.publish(detection_key, detections.serialize())) {
                std::lock_guard<std::mutex> lock(state_mutex);
                detection_count += detections.size();

                if (current_frame % 30 == 0) {
                    std::cout << "[Frame " << current_frame << "] "
                              << detections.size() << " objects detected";
                    if (!detections.empty()) {
                        const auto& det = detections[0];
                        std::cout << " - first at (" << det.world_x << ", "
                                  << det.world_y << ", " << det.world_z << ")m";
                    }
                    std::cout << std::endl;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[Error] " << e.what() << std::endl;
        }
    });

    std::cout << "Subscribed to: " << camera_key << std::endl;
    std::cout << "Publishing to: " << detection_key << std::endl;
    std::cout << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    // Main loop - just keep running
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << std::endl;
    std::cout << "Shutting down..." << std::endl;
    std::cout << "Total frames processed: " << frame_count << std::endl;
    std::cout << "Total detections: " << detection_count << std::endl;

    return 0;
}
