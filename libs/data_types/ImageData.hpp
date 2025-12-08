#pragma once

#include <cstdint>
#include <vector>
#include <opencv2/core.hpp>

namespace data_types {

// Image encoding types
enum class ImageEncoding : uint32_t {
    BGR8 = 0,    // 3-channel BGR (OpenCV default)
    RGB8 = 1,    // 3-channel RGB
    GRAY8 = 2,   // Single channel grayscale
    BGRA8 = 3,   // 4-channel BGRA
    RGBA8 = 4    // 4-channel RGBA
};

// Image data container with serialization support
// Binary format: [width:4][height:4][channels:4][encoding:4][pixel_data:...]
struct ImageData {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t channels = 0;
    ImageEncoding encoding = ImageEncoding::BGR8;
    std::vector<uint8_t> pixels;

    // Construct from raw parameters
    ImageData() = default;
    ImageData(uint32_t w, uint32_t h, uint32_t ch, ImageEncoding enc, std::vector<uint8_t> data);

    // Construct from OpenCV Mat
    static ImageData from_cv_mat(const cv::Mat& mat);

    // Convert to OpenCV Mat
    cv::Mat to_cv_mat() const;

    // Serialize to byte vector
    std::vector<uint8_t> serialize() const;

    // Deserialize from byte vector
    static ImageData deserialize(const std::vector<uint8_t>& data);

    // Validation
    bool is_valid() const;

    // Header size in bytes (4 uint32_t values)
    static constexpr size_t HEADER_SIZE = 16;
};

} // namespace data_types
