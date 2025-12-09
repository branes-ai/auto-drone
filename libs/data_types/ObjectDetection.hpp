#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

namespace data_types {

// Object detection result from perception system
// Contains both 2D bounding box (pixel coordinates) and estimated 3D world position
struct ObjectDetection {
    // Object identification
    uint32_t object_id = 0;        // Unique tracking ID
    uint32_t class_id = 0;         // Object class (0=unknown, 1=person, 2=vehicle, etc.)
    float confidence = 0.0f;       // Detection confidence [0.0, 1.0]

    // 2D bounding box in pixel coordinates (image space)
    float bbox_x_min = 0.0f;       // Left edge
    float bbox_y_min = 0.0f;       // Top edge
    float bbox_x_max = 0.0f;       // Right edge
    float bbox_y_max = 0.0f;       // Bottom edge

    // Estimated 3D world coordinates (NED frame, relative to drone)
    float world_x = 0.0f;          // Forward (meters)
    float world_y = 0.0f;          // Right (meters)
    float world_z = 0.0f;          // Down (meters)
    bool has_3d_position = false;  // True if 3D coordinates are valid

    // Timestamp
    uint64_t timestamp_us = 0;

    // Common object class IDs
    enum ClassId : uint32_t {
        UNKNOWN = 0,
        PERSON = 1,
        VEHICLE = 2,
        ANIMAL = 3,
        DRONE = 4,
        MARKER = 5,       // ArUco, AprilTag, etc.
        COLORED_BLOB = 6  // Simple color tracking target
    };

    ObjectDetection() = default;

    // Convenience constructor for 2D detection
    ObjectDetection(uint32_t obj_id, uint32_t cls_id, float conf,
                    float x_min, float y_min, float x_max, float y_max,
                    uint64_t ts = 0);

    // Convenience constructor with 3D position
    ObjectDetection(uint32_t obj_id, uint32_t cls_id, float conf,
                    float x_min, float y_min, float x_max, float y_max,
                    float wx, float wy, float wz,
                    uint64_t ts = 0);

    // Bounding box helpers
    float bbox_width() const { return bbox_x_max - bbox_x_min; }
    float bbox_height() const { return bbox_y_max - bbox_y_min; }
    float bbox_center_x() const { return (bbox_x_min + bbox_x_max) / 2.0f; }
    float bbox_center_y() const { return (bbox_y_min + bbox_y_max) / 2.0f; }
    float bbox_area() const { return bbox_width() * bbox_height(); }

    // Serialize to byte vector
    std::vector<uint8_t> serialize() const;

    // Deserialize from byte vector
    static ObjectDetection deserialize(const std::vector<uint8_t>& data);

    // Size in bytes when serialized
    static constexpr size_t SERIALIZED_SIZE = 56;
};

// List of detections from a single frame
struct ObjectDetectionList {
    std::vector<ObjectDetection> detections;
    uint64_t frame_timestamp_us = 0;  // Timestamp of source image
    uint32_t frame_id = 0;            // Frame number for tracking

    ObjectDetectionList() = default;

    void add(const ObjectDetection& det) { detections.push_back(det); }
    void clear() { detections.clear(); }
    size_t size() const { return detections.size(); }
    bool empty() const { return detections.empty(); }

    const ObjectDetection& operator[](size_t idx) const { return detections[idx]; }
    ObjectDetection& operator[](size_t idx) { return detections[idx]; }

    // Serialize to byte vector
    std::vector<uint8_t> serialize() const;

    // Deserialize from byte vector
    static ObjectDetectionList deserialize(const std::vector<uint8_t>& data);
};

} // namespace data_types
