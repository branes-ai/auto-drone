#pragma once

#include "ObjectDetection.hpp"
#include <opencv2/core.hpp>
#include <vector>
#include <cstdint>

namespace autonomy_stack {

// Camera intrinsic parameters for 2D-to-3D projection
struct CameraIntrinsics {
    float fx = 500.0f;   // Focal length X (pixels)
    float fy = 500.0f;   // Focal length Y (pixels)
    float cx = 320.0f;   // Principal point X (pixels)
    float cy = 240.0f;   // Principal point Y (pixels)
    int width = 640;     // Image width
    int height = 480;    // Image height

    // Create intrinsics for a given resolution with typical FOV
    static CameraIntrinsics from_resolution(int w, int h, float hfov_deg = 70.0f);
};

// Base class for object detectors
class ObjectDetector {
public:
    virtual ~ObjectDetector() = default;

    // Detect objects in an image
    // Returns list of detections (2D bounding boxes only)
    virtual std::vector<data_types::ObjectDetection> detect(const cv::Mat& image) = 0;

    // Get detector name for logging
    virtual std::string name() const = 0;
};

// Simple color-based blob detector
// Detects objects of a specified HSV color range
class ColorBlobDetector : public ObjectDetector {
public:
    struct Config {
        // HSV range for target color (default: bright red)
        int h_min = 0;
        int h_max = 10;
        int s_min = 100;
        int s_max = 255;
        int v_min = 100;
        int v_max = 255;

        // Also detect wrap-around red (170-180)
        bool detect_red_wraparound = true;

        // Minimum blob area in pixels
        int min_area = 100;

        // Maximum number of detections to return
        int max_detections = 10;
    };

    ColorBlobDetector() = default;
    explicit ColorBlobDetector(const Config& config);

    void set_config(const Config& config) { config_ = config; }
    const Config& config() const { return config_; }

    std::vector<data_types::ObjectDetection> detect(const cv::Mat& image) override;
    std::string name() const override { return "ColorBlobDetector"; }

private:
    Config config_;
    uint32_t next_id_ = 1;
};

// PerceptionEngine: Combines detection with 2D-to-3D projection
// Uses camera intrinsics and optional depth information to estimate world coordinates
class PerceptionEngine {
public:
    struct Config {
        CameraIntrinsics camera;
        float default_object_depth = 5.0f;  // Default depth when no depth info available (meters)
        bool estimate_depth_from_size = true;  // Estimate depth from bounding box size
        float reference_object_width = 0.5f;   // Reference object width for depth estimation (meters)
        float reference_bbox_width = 100.0f;   // Expected bbox width at 1m distance (pixels)
    };

    PerceptionEngine();
    explicit PerceptionEngine(const Config& config);

    void set_config(const Config& config) { config_ = config; }
    const Config& config() const { return config_; }

    // Set the object detector to use
    void set_detector(std::unique_ptr<ObjectDetector> detector);

    // Process an image and return detections with 3D positions
    data_types::ObjectDetectionList process(const cv::Mat& image, uint32_t frame_id = 0);

    // Process an image with depth map
    data_types::ObjectDetectionList process(const cv::Mat& image, const cv::Mat& depth, uint32_t frame_id = 0);

    // Project 2D pixel coordinates to 3D ray direction (normalized)
    void pixel_to_ray(float px, float py, float& rx, float& ry, float& rz) const;

    // Estimate 3D position from bounding box center and depth
    void estimate_3d_position(const data_types::ObjectDetection& det, float depth,
                              float& wx, float& wy, float& wz) const;

    // Estimate depth from bounding box size (assumes known object size)
    float estimate_depth_from_bbox(float bbox_width) const;

private:
    Config config_;
    std::unique_ptr<ObjectDetector> detector_;
};

} // namespace autonomy_stack
