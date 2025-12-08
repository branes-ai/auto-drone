#include "ImageData.hpp"
#include <cstring>
#include <stdexcept>

namespace data_types {

ImageData::ImageData(uint32_t w, uint32_t h, uint32_t ch, ImageEncoding enc, std::vector<uint8_t> data)
    : width(w), height(h), channels(ch), encoding(enc), pixels(std::move(data)) {}

ImageData ImageData::from_cv_mat(const cv::Mat& mat) {
    if (mat.empty()) {
        return ImageData{};
    }

    ImageData img;
    img.width = static_cast<uint32_t>(mat.cols);
    img.height = static_cast<uint32_t>(mat.rows);
    img.channels = static_cast<uint32_t>(mat.channels());

    // Determine encoding based on OpenCV type
    switch (mat.channels()) {
        case 1:
            img.encoding = ImageEncoding::GRAY8;
            break;
        case 3:
            img.encoding = ImageEncoding::BGR8;  // OpenCV default
            break;
        case 4:
            img.encoding = ImageEncoding::BGRA8;
            break;
        default:
            img.encoding = ImageEncoding::BGR8;
    }

    // Copy pixel data (ensure continuous)
    cv::Mat continuous_mat = mat.isContinuous() ? mat : mat.clone();
    size_t data_size = continuous_mat.total() * continuous_mat.elemSize();
    img.pixels.resize(data_size);
    std::memcpy(img.pixels.data(), continuous_mat.data, data_size);

    return img;
}

cv::Mat ImageData::to_cv_mat() const {
    if (!is_valid()) {
        return cv::Mat{};
    }

    int cv_type;
    switch (channels) {
        case 1: cv_type = CV_8UC1; break;
        case 3: cv_type = CV_8UC3; break;
        case 4: cv_type = CV_8UC4; break;
        default: return cv::Mat{};
    }

    cv::Mat mat(static_cast<int>(height), static_cast<int>(width), cv_type);
    std::memcpy(mat.data, pixels.data(), pixels.size());

    return mat;
}

std::vector<uint8_t> ImageData::serialize() const {
    std::vector<uint8_t> result;
    result.reserve(HEADER_SIZE + pixels.size());

    // Write header (little-endian)
    auto write_u32 = [&result](uint32_t value) {
        result.push_back(static_cast<uint8_t>(value & 0xFF));
        result.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
        result.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
        result.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
    };

    write_u32(width);
    write_u32(height);
    write_u32(channels);
    write_u32(static_cast<uint32_t>(encoding));

    // Append pixel data
    result.insert(result.end(), pixels.begin(), pixels.end());

    return result;
}

ImageData ImageData::deserialize(const std::vector<uint8_t>& data) {
    if (data.size() < HEADER_SIZE) {
        throw std::runtime_error("ImageData::deserialize: data too small for header");
    }

    auto read_u32 = [&data](size_t offset) -> uint32_t {
        return static_cast<uint32_t>(data[offset]) |
               (static_cast<uint32_t>(data[offset + 1]) << 8) |
               (static_cast<uint32_t>(data[offset + 2]) << 16) |
               (static_cast<uint32_t>(data[offset + 3]) << 24);
    };

    ImageData img;
    img.width = read_u32(0);
    img.height = read_u32(4);
    img.channels = read_u32(8);
    img.encoding = static_cast<ImageEncoding>(read_u32(12));

    // Calculate expected pixel data size
    size_t expected_size = static_cast<size_t>(img.width) * img.height * img.channels;
    size_t available_size = data.size() - HEADER_SIZE;

    if (available_size < expected_size) {
        throw std::runtime_error("ImageData::deserialize: insufficient pixel data");
    }

    // Copy pixel data
    img.pixels.assign(data.begin() + HEADER_SIZE, data.begin() + HEADER_SIZE + expected_size);

    return img;
}

bool ImageData::is_valid() const {
    if (width == 0 || height == 0 || channels == 0) {
        return false;
    }
    size_t expected_size = static_cast<size_t>(width) * height * channels;
    return pixels.size() == expected_size;
}

} // namespace data_types
