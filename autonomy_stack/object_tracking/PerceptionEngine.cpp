#include "PerceptionEngine.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <chrono>
#include <algorithm>

namespace autonomy_stack {

// --- CameraIntrinsics ---

CameraIntrinsics CameraIntrinsics::from_resolution(int w, int h, float hfov_deg) {
    CameraIntrinsics intrinsics;
    intrinsics.width = w;
    intrinsics.height = h;
    intrinsics.cx = w / 2.0f;
    intrinsics.cy = h / 2.0f;

    // Calculate focal length from horizontal FOV
    float hfov_rad = hfov_deg * 3.14159265f / 180.0f;
    intrinsics.fx = (w / 2.0f) / std::tan(hfov_rad / 2.0f);
    intrinsics.fy = intrinsics.fx;  // Assume square pixels

    return intrinsics;
}

// --- ColorBlobDetector ---

ColorBlobDetector::ColorBlobDetector(const Config& config)
    : config_(config) {}

std::vector<data_types::ObjectDetection> ColorBlobDetector::detect(const cv::Mat& image) {
    std::vector<data_types::ObjectDetection> detections;

    if (image.empty()) {
        return detections;
    }

    // Convert to HSV
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // Create mask for target color
    cv::Mat mask;
    cv::inRange(hsv,
                cv::Scalar(config_.h_min, config_.s_min, config_.v_min),
                cv::Scalar(config_.h_max, config_.s_max, config_.v_max),
                mask);

    // Handle red color wraparound (170-180 range)
    if (config_.detect_red_wraparound && config_.h_min < 20) {
        cv::Mat mask2;
        cv::inRange(hsv,
                    cv::Scalar(170, config_.s_min, config_.v_min),
                    cv::Scalar(180, config_.s_max, config_.v_max),
                    mask2);
        cv::bitwise_or(mask, mask2, mask);
    }

    // Morphological operations to clean up mask
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Sort by area (largest first)
    std::sort(contours.begin(), contours.end(),
              [](const auto& a, const auto& b) {
                  return cv::contourArea(a) > cv::contourArea(b);
              });

    // Get timestamp
    auto now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // Process each contour
    int count = 0;
    for (const auto& contour : contours) {
        if (count >= config_.max_detections) break;

        double area = cv::contourArea(contour);
        if (area < config_.min_area) continue;

        cv::Rect bbox = cv::boundingRect(contour);

        // Calculate confidence based on how "blob-like" the contour is
        double perimeter = cv::arcLength(contour, true);
        double circularity = 4 * 3.14159265 * area / (perimeter * perimeter);
        float confidence = static_cast<float>(std::min(1.0, circularity * 1.5));

        data_types::ObjectDetection det;
        det.object_id = next_id_++;
        det.class_id = data_types::ObjectDetection::COLORED_BLOB;
        det.confidence = confidence;
        det.bbox_x_min = static_cast<float>(bbox.x);
        det.bbox_y_min = static_cast<float>(bbox.y);
        det.bbox_x_max = static_cast<float>(bbox.x + bbox.width);
        det.bbox_y_max = static_cast<float>(bbox.y + bbox.height);
        det.has_3d_position = false;
        det.timestamp_us = now;

        detections.push_back(det);
        count++;
    }

    return detections;
}

// --- PerceptionEngine ---

PerceptionEngine::PerceptionEngine() {
    detector_ = std::make_unique<ColorBlobDetector>();
}

PerceptionEngine::PerceptionEngine(const Config& config)
    : config_(config) {
    detector_ = std::make_unique<ColorBlobDetector>();
}

void PerceptionEngine::set_detector(std::unique_ptr<ObjectDetector> detector) {
    detector_ = std::move(detector);
}

data_types::ObjectDetectionList PerceptionEngine::process(const cv::Mat& image, uint32_t frame_id) {
    data_types::ObjectDetectionList result;
    result.frame_id = frame_id;
    result.frame_timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    if (!detector_ || image.empty()) {
        return result;
    }

    // Run detection
    auto detections = detector_->detect(image);

    // Add 3D position estimates
    for (auto& det : detections) {
        float depth = config_.default_object_depth;

        if (config_.estimate_depth_from_size) {
            depth = estimate_depth_from_bbox(det.bbox_width());
        }

        float wx, wy, wz;
        estimate_3d_position(det, depth, wx, wy, wz);
        det.world_x = wx;
        det.world_y = wy;
        det.world_z = wz;
        det.has_3d_position = true;

        result.add(det);
    }

    return result;
}

data_types::ObjectDetectionList PerceptionEngine::process(const cv::Mat& image,
                                                           const cv::Mat& depth,
                                                           uint32_t frame_id) {
    data_types::ObjectDetectionList result;
    result.frame_id = frame_id;
    result.frame_timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    if (!detector_ || image.empty()) {
        return result;
    }

    // Run detection
    auto detections = detector_->detect(image);

    // Add 3D position estimates using depth map
    for (auto& det : detections) {
        float depth_val = config_.default_object_depth;

        if (!depth.empty()) {
            // Sample depth at bounding box center
            int cx = static_cast<int>(det.bbox_center_x());
            int cy = static_cast<int>(det.bbox_center_y());
            cx = std::clamp(cx, 0, depth.cols - 1);
            cy = std::clamp(cy, 0, depth.rows - 1);

            if (depth.type() == CV_32F) {
                depth_val = depth.at<float>(cy, cx);
            } else if (depth.type() == CV_16U) {
                // Assume depth in millimeters
                depth_val = depth.at<uint16_t>(cy, cx) / 1000.0f;
            }

            // Validate depth
            if (depth_val <= 0.0f || depth_val > 100.0f) {
                depth_val = config_.default_object_depth;
            }
        }

        float wx, wy, wz;
        estimate_3d_position(det, depth_val, wx, wy, wz);
        det.world_x = wx;
        det.world_y = wy;
        det.world_z = wz;
        det.has_3d_position = true;

        result.add(det);
    }

    return result;
}

void PerceptionEngine::pixel_to_ray(float px, float py, float& rx, float& ry, float& rz) const {
    // Convert pixel coordinates to normalized camera coordinates
    // Then to a ray direction in camera frame (Z forward, X right, Y down)
    float nx = (px - config_.camera.cx) / config_.camera.fx;
    float ny = (py - config_.camera.cy) / config_.camera.fy;

    // Normalize to unit vector
    float len = std::sqrt(nx * nx + ny * ny + 1.0f);
    rx = nx / len;  // Right
    ry = ny / len;  // Down
    rz = 1.0f / len; // Forward
}

void PerceptionEngine::estimate_3d_position(const data_types::ObjectDetection& det,
                                             float depth,
                                             float& wx, float& wy, float& wz) const {
    // Get ray direction for bounding box center
    float rx, ry, rz;
    pixel_to_ray(det.bbox_center_x(), det.bbox_center_y(), rx, ry, rz);

    // Scale ray by depth to get 3D position
    // Convert from camera frame (Z forward, X right, Y down) to NED (X forward, Y right, Z down)
    wx = depth * rz;  // Forward -> X
    wy = depth * rx;  // Right -> Y
    wz = depth * ry;  // Down -> Z
}

float PerceptionEngine::estimate_depth_from_bbox(float bbox_width) const {
    if (bbox_width <= 0.0f) {
        return config_.default_object_depth;
    }

    // Using similar triangles:
    // bbox_width / fx = object_width / depth
    // depth = object_width * fx / bbox_width
    //
    // Alternatively, using reference measurements:
    // depth = reference_distance * (reference_bbox_width / bbox_width)
    float depth = config_.reference_object_width * config_.camera.fx / bbox_width;

    // Clamp to reasonable range
    depth = std::clamp(depth, 0.5f, 50.0f);

    return depth;
}

} // namespace autonomy_stack
