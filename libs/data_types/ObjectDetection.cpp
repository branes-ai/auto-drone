#include "ObjectDetection.hpp"
#include <stdexcept>
#include <cstring>

namespace data_types {

// --- ObjectDetection ---

ObjectDetection::ObjectDetection(uint32_t obj_id, uint32_t cls_id, float conf,
                                 float x_min, float y_min, float x_max, float y_max,
                                 uint64_t ts)
    : object_id(obj_id), class_id(cls_id), confidence(conf),
      bbox_x_min(x_min), bbox_y_min(y_min), bbox_x_max(x_max), bbox_y_max(y_max),
      has_3d_position(false), timestamp_us(ts) {}

ObjectDetection::ObjectDetection(uint32_t obj_id, uint32_t cls_id, float conf,
                                 float x_min, float y_min, float x_max, float y_max,
                                 float wx, float wy, float wz,
                                 uint64_t ts)
    : object_id(obj_id), class_id(cls_id), confidence(conf),
      bbox_x_min(x_min), bbox_y_min(y_min), bbox_x_max(x_max), bbox_y_max(y_max),
      world_x(wx), world_y(wy), world_z(wz), has_3d_position(true),
      timestamp_us(ts) {}

std::vector<uint8_t> ObjectDetection::serialize() const {
    std::vector<uint8_t> data(SERIALIZED_SIZE, 0);
    size_t offset = 0;

    std::memcpy(data.data() + offset, &object_id, sizeof(object_id)); offset += sizeof(object_id);
    std::memcpy(data.data() + offset, &class_id, sizeof(class_id)); offset += sizeof(class_id);
    std::memcpy(data.data() + offset, &confidence, sizeof(confidence)); offset += sizeof(confidence);
    std::memcpy(data.data() + offset, &bbox_x_min, sizeof(bbox_x_min)); offset += sizeof(bbox_x_min);
    std::memcpy(data.data() + offset, &bbox_y_min, sizeof(bbox_y_min)); offset += sizeof(bbox_y_min);
    std::memcpy(data.data() + offset, &bbox_x_max, sizeof(bbox_x_max)); offset += sizeof(bbox_x_max);
    std::memcpy(data.data() + offset, &bbox_y_max, sizeof(bbox_y_max)); offset += sizeof(bbox_y_max);
    std::memcpy(data.data() + offset, &world_x, sizeof(world_x)); offset += sizeof(world_x);
    std::memcpy(data.data() + offset, &world_y, sizeof(world_y)); offset += sizeof(world_y);
    std::memcpy(data.data() + offset, &world_z, sizeof(world_z)); offset += sizeof(world_z);
    uint8_t flags = has_3d_position ? 1 : 0;
    std::memcpy(data.data() + offset, &flags, sizeof(flags)); offset += sizeof(flags);
    // 3 bytes padding
    offset += 3;
    std::memcpy(data.data() + offset, &timestamp_us, sizeof(timestamp_us));

    return data;
}

ObjectDetection ObjectDetection::deserialize(const std::vector<uint8_t>& data) {
    if (data.size() < SERIALIZED_SIZE) {
        throw std::runtime_error("ObjectDetection: insufficient data for deserialization");
    }

    ObjectDetection det;
    size_t offset = 0;

    std::memcpy(&det.object_id, data.data() + offset, sizeof(det.object_id)); offset += sizeof(det.object_id);
    std::memcpy(&det.class_id, data.data() + offset, sizeof(det.class_id)); offset += sizeof(det.class_id);
    std::memcpy(&det.confidence, data.data() + offset, sizeof(det.confidence)); offset += sizeof(det.confidence);
    std::memcpy(&det.bbox_x_min, data.data() + offset, sizeof(det.bbox_x_min)); offset += sizeof(det.bbox_x_min);
    std::memcpy(&det.bbox_y_min, data.data() + offset, sizeof(det.bbox_y_min)); offset += sizeof(det.bbox_y_min);
    std::memcpy(&det.bbox_x_max, data.data() + offset, sizeof(det.bbox_x_max)); offset += sizeof(det.bbox_x_max);
    std::memcpy(&det.bbox_y_max, data.data() + offset, sizeof(det.bbox_y_max)); offset += sizeof(det.bbox_y_max);
    std::memcpy(&det.world_x, data.data() + offset, sizeof(det.world_x)); offset += sizeof(det.world_x);
    std::memcpy(&det.world_y, data.data() + offset, sizeof(det.world_y)); offset += sizeof(det.world_y);
    std::memcpy(&det.world_z, data.data() + offset, sizeof(det.world_z)); offset += sizeof(det.world_z);
    uint8_t flags;
    std::memcpy(&flags, data.data() + offset, sizeof(flags)); offset += sizeof(flags);
    det.has_3d_position = (flags & 1) != 0;
    offset += 3; // padding
    std::memcpy(&det.timestamp_us, data.data() + offset, sizeof(det.timestamp_us));

    return det;
}

// --- ObjectDetectionList ---

std::vector<uint8_t> ObjectDetectionList::serialize() const {
    // Header: count (4 bytes) + frame_id (4 bytes) + frame_timestamp (8 bytes)
    size_t header_size = sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint64_t);
    size_t total_size = header_size + detections.size() * ObjectDetection::SERIALIZED_SIZE;
    std::vector<uint8_t> data(total_size);

    size_t offset = 0;
    uint32_t count = static_cast<uint32_t>(detections.size());
    std::memcpy(data.data() + offset, &count, sizeof(count)); offset += sizeof(count);
    std::memcpy(data.data() + offset, &frame_id, sizeof(frame_id)); offset += sizeof(frame_id);
    std::memcpy(data.data() + offset, &frame_timestamp_us, sizeof(frame_timestamp_us)); offset += sizeof(frame_timestamp_us);

    for (const auto& det : detections) {
        auto det_data = det.serialize();
        std::memcpy(data.data() + offset, det_data.data(), det_data.size());
        offset += det_data.size();
    }

    return data;
}

ObjectDetectionList ObjectDetectionList::deserialize(const std::vector<uint8_t>& data) {
    size_t header_size = sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint64_t);
    if (data.size() < header_size) {
        throw std::runtime_error("ObjectDetectionList: insufficient data for header");
    }

    ObjectDetectionList list;
    size_t offset = 0;

    uint32_t count;
    std::memcpy(&count, data.data() + offset, sizeof(count)); offset += sizeof(count);
    std::memcpy(&list.frame_id, data.data() + offset, sizeof(list.frame_id)); offset += sizeof(list.frame_id);
    std::memcpy(&list.frame_timestamp_us, data.data() + offset, sizeof(list.frame_timestamp_us)); offset += sizeof(list.frame_timestamp_us);

    size_t expected_size = header_size + count * ObjectDetection::SERIALIZED_SIZE;
    if (data.size() < expected_size) {
        throw std::runtime_error("ObjectDetectionList: insufficient data for detections");
    }

    list.detections.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        std::vector<uint8_t> det_data(data.begin() + offset,
                                       data.begin() + offset + ObjectDetection::SERIALIZED_SIZE);
        list.detections.push_back(ObjectDetection::deserialize(det_data));
        offset += ObjectDetection::SERIALIZED_SIZE;
    }

    return list;
}

} // namespace data_types
