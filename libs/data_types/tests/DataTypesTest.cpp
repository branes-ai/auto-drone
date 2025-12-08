#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "ImageData.hpp"
#include "Odometry.hpp"

using namespace data_types;
using Catch::Matchers::WithinAbs;

// ============================================================================
// ImageData Tests
// ============================================================================

TEST_CASE("ImageData default construction", "[ImageData]") {
    ImageData img;
    REQUIRE(img.width == 0);
    REQUIRE(img.height == 0);
    REQUIRE(img.channels == 0);
    REQUIRE(img.pixels.empty());
    REQUIRE_FALSE(img.is_valid());
}

TEST_CASE("ImageData parameterized construction", "[ImageData]") {
    std::vector<uint8_t> pixels(100 * 100 * 3, 128);
    ImageData img(100, 100, 3, ImageEncoding::BGR8, pixels);

    REQUIRE(img.width == 100);
    REQUIRE(img.height == 100);
    REQUIRE(img.channels == 3);
    REQUIRE(img.encoding == ImageEncoding::BGR8);
    REQUIRE(img.pixels.size() == 100 * 100 * 3);
    REQUIRE(img.is_valid());
}

TEST_CASE("ImageData serialization roundtrip", "[ImageData]") {
    // Create test image
    std::vector<uint8_t> pixels(64 * 48 * 3);
    for (size_t i = 0; i < pixels.size(); ++i) {
        pixels[i] = static_cast<uint8_t>(i % 256);
    }
    ImageData original(64, 48, 3, ImageEncoding::RGB8, pixels);

    // Serialize
    auto serialized = original.serialize();
    REQUIRE(serialized.size() == ImageData::HEADER_SIZE + pixels.size());

    // Deserialize
    auto restored = ImageData::deserialize(serialized);

    // Verify
    REQUIRE(restored.width == original.width);
    REQUIRE(restored.height == original.height);
    REQUIRE(restored.channels == original.channels);
    REQUIRE(restored.encoding == original.encoding);
    REQUIRE(restored.pixels == original.pixels);
}

TEST_CASE("ImageData grayscale image", "[ImageData]") {
    std::vector<uint8_t> pixels(32 * 32, 200);
    ImageData img(32, 32, 1, ImageEncoding::GRAY8, pixels);

    REQUIRE(img.is_valid());

    auto serialized = img.serialize();
    auto restored = ImageData::deserialize(serialized);

    REQUIRE(restored.channels == 1);
    REQUIRE(restored.encoding == ImageEncoding::GRAY8);
    REQUIRE(restored.pixels == pixels);
}

TEST_CASE("ImageData deserialization fails on insufficient data", "[ImageData]") {
    std::vector<uint8_t> too_small(10);
    REQUIRE_THROWS_AS(ImageData::deserialize(too_small), std::runtime_error);
}

TEST_CASE("ImageData deserialization fails on truncated pixel data", "[ImageData]") {
    // Create header claiming 100x100 image but provide no pixel data
    std::vector<uint8_t> bad_data(ImageData::HEADER_SIZE);
    // width = 100 (little endian)
    bad_data[0] = 100; bad_data[1] = 0; bad_data[2] = 0; bad_data[3] = 0;
    // height = 100
    bad_data[4] = 100; bad_data[5] = 0; bad_data[6] = 0; bad_data[7] = 0;
    // channels = 3
    bad_data[8] = 3; bad_data[9] = 0; bad_data[10] = 0; bad_data[11] = 0;
    // encoding = 0
    bad_data[12] = 0; bad_data[13] = 0; bad_data[14] = 0; bad_data[15] = 0;

    REQUIRE_THROWS_AS(ImageData::deserialize(bad_data), std::runtime_error);
}

TEST_CASE("ImageData OpenCV Mat conversion", "[ImageData]") {
    // Create a small BGR image
    cv::Mat mat(10, 20, CV_8UC3, cv::Scalar(100, 150, 200));

    auto img = ImageData::from_cv_mat(mat);
    REQUIRE(img.width == 20);
    REQUIRE(img.height == 10);
    REQUIRE(img.channels == 3);
    REQUIRE(img.is_valid());

    // Convert back
    cv::Mat restored = img.to_cv_mat();
    REQUIRE(restored.cols == 20);
    REQUIRE(restored.rows == 10);
    REQUIRE(restored.channels() == 3);

    // Check pixel values match
    cv::Vec3b original_pixel = mat.at<cv::Vec3b>(5, 10);
    cv::Vec3b restored_pixel = restored.at<cv::Vec3b>(5, 10);
    REQUIRE(original_pixel == restored_pixel);
}

TEST_CASE("ImageData from empty Mat returns invalid", "[ImageData]") {
    cv::Mat empty_mat;
    auto img = ImageData::from_cv_mat(empty_mat);
    REQUIRE_FALSE(img.is_valid());
}

// ============================================================================
// Odometry Tests
// ============================================================================

TEST_CASE("Odometry default construction", "[Odometry]") {
    Odometry odom;
    REQUIRE(odom.x == 0.0f);
    REQUIRE(odom.y == 0.0f);
    REQUIRE(odom.z == 0.0f);
    REQUIRE(odom.roll == 0.0f);
    REQUIRE(odom.pitch == 0.0f);
    REQUIRE(odom.yaw == 0.0f);
    REQUIRE(odom.timestamp_us == 0);
}

TEST_CASE("Odometry parameterized construction", "[Odometry]") {
    Odometry odom(1.5f, -2.3f, 10.0f, 0.1f, 0.2f, 3.14f, 1234567890ULL);

    REQUIRE_THAT(odom.x, WithinAbs(1.5f, 0.001f));
    REQUIRE_THAT(odom.y, WithinAbs(-2.3f, 0.001f));
    REQUIRE_THAT(odom.z, WithinAbs(10.0f, 0.001f));
    REQUIRE_THAT(odom.roll, WithinAbs(0.1f, 0.001f));
    REQUIRE_THAT(odom.pitch, WithinAbs(0.2f, 0.001f));
    REQUIRE_THAT(odom.yaw, WithinAbs(3.14f, 0.001f));
    REQUIRE(odom.timestamp_us == 1234567890ULL);
}

TEST_CASE("Odometry serialization roundtrip", "[Odometry]") {
    Odometry original(1.5f, -2.3f, 10.0f, 0.1f, 0.2f, 3.14159f, 9876543210ULL);

    // Serialize
    auto serialized = original.serialize();
    REQUIRE(serialized.size() == Odometry::SERIALIZED_SIZE);

    // Deserialize
    auto restored = Odometry::deserialize(serialized);

    // Verify (using floating point comparison)
    REQUIRE_THAT(restored.x, WithinAbs(original.x, 0.0001f));
    REQUIRE_THAT(restored.y, WithinAbs(original.y, 0.0001f));
    REQUIRE_THAT(restored.z, WithinAbs(original.z, 0.0001f));
    REQUIRE_THAT(restored.roll, WithinAbs(original.roll, 0.0001f));
    REQUIRE_THAT(restored.pitch, WithinAbs(original.pitch, 0.0001f));
    REQUIRE_THAT(restored.yaw, WithinAbs(original.yaw, 0.0001f));
    REQUIRE(restored.timestamp_us == original.timestamp_us);
}

TEST_CASE("Odometry serialization with negative values", "[Odometry]") {
    Odometry original(-100.5f, -200.25f, -0.001f, -3.14f, -1.57f, -0.785f, 0);

    auto serialized = original.serialize();
    auto restored = Odometry::deserialize(serialized);

    REQUIRE_THAT(restored.x, WithinAbs(original.x, 0.0001f));
    REQUIRE_THAT(restored.y, WithinAbs(original.y, 0.0001f));
    REQUIRE_THAT(restored.z, WithinAbs(original.z, 0.0001f));
    REQUIRE_THAT(restored.roll, WithinAbs(original.roll, 0.0001f));
    REQUIRE_THAT(restored.pitch, WithinAbs(original.pitch, 0.0001f));
    REQUIRE_THAT(restored.yaw, WithinAbs(original.yaw, 0.0001f));
}

TEST_CASE("Odometry serialization with large timestamp", "[Odometry]") {
    // Test with max uint64 value
    Odometry original(0, 0, 0, 0, 0, 0, UINT64_MAX);

    auto serialized = original.serialize();
    auto restored = Odometry::deserialize(serialized);

    REQUIRE(restored.timestamp_us == UINT64_MAX);
}

TEST_CASE("Odometry deserialization fails on insufficient data", "[Odometry]") {
    std::vector<uint8_t> too_small(20);
    REQUIRE_THROWS_AS(Odometry::deserialize(too_small), std::runtime_error);
}

TEST_CASE("Odometry serialized size is correct", "[Odometry]") {
    REQUIRE(Odometry::SERIALIZED_SIZE == 32);

    Odometry odom;
    auto serialized = odom.serialize();
    REQUIRE(serialized.size() == 32);
}
