#include "AirSimClient.hpp"

// Project AirSim includes
// These headers are from: https://github.com/iamaisim/ProjectAirSim
// Path: client/cpp/AirSimClientDLL/Include/AirSimClient/
#ifdef PROJECTAIRSIM_AVAILABLE
#include <AirSimClient/Client.h>
#include <AirSimClient/World.h>
#include <AirSimClient/Drone.h>
#include <AirSimClient/Types.h>
#endif

#include <cmath>
#include <iostream>
#include <thread>

namespace sim_interfaces {

// Helper: Convert quaternion to Euler angles (roll, pitch, yaw)
static void quaternion_to_euler(float qw, float qx, float qy, float qz,
                                 float& roll, float& pitch, float& yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0f) {
        pitch = std::copysign(3.14159265f / 2.0f, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

AirSimClient::AirSimClient() = default;

AirSimClient::AirSimClient(const AirSimConfig& config)
    : config_(config) {}

AirSimClient::~AirSimClient() {
    disconnect();
}

AirSimClient::AirSimClient(AirSimClient&&) noexcept = default;
AirSimClient& AirSimClient::operator=(AirSimClient&&) noexcept = default;

bool AirSimClient::connect() {
#ifdef PROJECTAIRSIM_AVAILABLE
    try {
        using namespace microsoft::projectairsim::client;

        // Create client and connect
        client_ = std::make_shared<Client>();

        // Project AirSim requires explicit connect() call
        // Connection string format: "host:port"
        std::string connection = config_.host + ":" + std::to_string(config_.port);
        if (!client_->Connect(connection.c_str())) {
            last_error_ = "Failed to connect to Project AirSim at " + connection;
            return false;
        }

        // Get world reference
        world_ = client_->GetWorld();
        if (!world_) {
            last_error_ = "Failed to get World reference";
            disconnect();
            return false;
        }

        // Get drone reference
        drone_ = world_->GetDrone(config_.vehicle_name.c_str());
        if (!drone_) {
            last_error_ = "Failed to get Drone '" + config_.vehicle_name + "'";
            disconnect();
            return false;
        }

        connected_ = true;
        last_error_.clear();

        if (config_.enable_api_control) {
            enable_api_control();
        }

        return true;
    } catch (const std::exception& e) {
        last_error_ = std::string("Connection failed: ") + e.what();
        connected_ = false;
        return false;
    }
#else
    last_error_ = "Project AirSim library not available. Build with PROJECTAIRSIM_ROOT set.";
    return false;
#endif
}

void AirSimClient::disconnect() {
    if (connected_) {
        try {
            disable_api_control();
        } catch (...) {
            // Ignore errors during disconnect
        }
    }
    drone_.reset();
    world_.reset();
    client_.reset();
    connected_ = false;
    api_control_enabled_ = false;
}

bool AirSimClient::is_connected() const {
    return connected_ && client_ != nullptr;
}

void AirSimClient::set_config(const AirSimConfig& config) {
    config_ = config;
}

bool AirSimClient::enable_api_control() {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;
    try {
        drone_->EnableAPIControl();
        api_control_enabled_ = true;
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}

bool AirSimClient::disable_api_control() {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;
    try {
        drone_->DisableAPIControl();
        api_control_enabled_ = false;
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}

bool AirSimClient::is_api_control_enabled() const {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;
    try {
        return drone_->IsAPIControlEnabled();
    } catch (...) {
        return false;
    }
#else
    return api_control_enabled_;
#endif
}

bool AirSimClient::arm() {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;
    try {
        // Project AirSim uses separate Arm() method (not armDisarm)
        drone_->Arm();
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}

bool AirSimClient::disarm() {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;
    try {
        drone_->Disarm();
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}

bool AirSimClient::can_arm() const {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;
    try {
        return drone_->CanArm();
    } catch (...) {
        return false;
    }
#else
    return false;
#endif
}

bool AirSimClient::takeoff(float timeout_sec) {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;
    try {
        // Project AirSim async methods use callbacks or can be waited on
        auto result = drone_->TakeoffAsync(timeout_sec, nullptr);
        // Wait for completion (blocking)
        // The actual wait mechanism depends on Project AirSim's AsyncResult API
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}

bool AirSimClient::land(float timeout_sec) {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;
    try {
        auto result = drone_->LandAsync(timeout_sec, nullptr);
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}

bool AirSimClient::hover() {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;
    try {
        drone_->HoverAsync();
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}

bool AirSimClient::go_home() {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;
    try {
        drone_->GoHomeAsync();
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}

data_types::ImageData AirSimClient::capture_rgb() {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return data_types::ImageData();

    try {
        // Project AirSim image capture API
        // Note: Exact API depends on final Project AirSim release
        // This is a placeholder based on documented structure

        // TODO: Implement when Project AirSim image API is finalized
        // Expected pattern:
        // auto image = drone_->GetCameraImage(config_.camera_name, ImageType::Scene);
        // return convert_image(image.data, image.width, image.height, 3);

        last_error_ = "Image capture not yet implemented for Project AirSim";
        return data_types::ImageData();
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return data_types::ImageData();
    }
#else
    return data_types::ImageData();
#endif
}

data_types::ImageData AirSimClient::capture_depth() {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return data_types::ImageData();

    try {
        // TODO: Implement when Project AirSim image API is finalized
        last_error_ = "Depth capture not yet implemented for Project AirSim";
        return data_types::ImageData();
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return data_types::ImageData();
    }
#else
    return data_types::ImageData();
#endif
}

std::vector<data_types::ImageData> AirSimClient::capture_all() {
    std::vector<data_types::ImageData> results;

    if (config_.capture_rgb) {
        auto rgb = capture_rgb();
        if (rgb.is_valid()) {
            results.push_back(std::move(rgb));
        }
    }

    if (config_.capture_depth) {
        auto depth = capture_depth();
        if (depth.is_valid()) {
            results.push_back(std::move(depth));
        }
    }

    return results;
}

data_types::Odometry AirSimClient::get_odometry() {
    data_types::Odometry odom;

#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return odom;

    try {
        // Get pose using Project AirSim API
        microsoft::projectairsim::client::Transform transform;
        drone_->GetGroundTruthPose(&transform);

        odom.x = transform.position.x;
        odom.y = transform.position.y;
        odom.z = transform.position.z;

        // Convert quaternion to Euler (Project AirSim uses radians)
        quaternion_to_euler(
            transform.orientation.w,
            transform.orientation.x,
            transform.orientation.y,
            transform.orientation.z,
            odom.roll, odom.pitch, odom.yaw);

        odom.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        return odom;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return odom;
    }
#else
    return odom;
#endif
}

AirSimClient::Pose AirSimClient::get_pose() {
    Pose pose = {0, 0, 0, 1, 0, 0, 0};

#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return pose;

    try {
        microsoft::projectairsim::client::Transform transform;
        drone_->GetGroundTruthPose(&transform);

        pose.x = transform.position.x;
        pose.y = transform.position.y;
        pose.z = transform.position.z;
        pose.qw = transform.orientation.w;
        pose.qx = transform.orientation.x;
        pose.qy = transform.orientation.y;
        pose.qz = transform.orientation.z;

        return pose;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return pose;
    }
#else
    return pose;
#endif
}

AirSimClient::Velocity AirSimClient::get_velocity() {
    Velocity vel = {0, 0, 0, 0, 0, 0};

#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return vel;

    try {
        // TODO: Get velocity from Project AirSim
        // Exact API depends on final release
        return vel;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return vel;
    }
#else
    return vel;
#endif
}

bool AirSimClient::move_by_velocity(float vx, float vy, float vz, float duration_sec) {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;

    try {
        // Project AirSim MoveByVelocityAsync
        // Parameters: v_north, v_east, v_down, duration, yaw_mode, yaw_is_rate, yaw, callback
        drone_->MoveByVelocityAsync(
            vx, vy, vz, duration_sec,
            microsoft::projectairsim::client::YawControlMode::MaxDegreeOfFreedom,
            false, 0.0f, nullptr);
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}

bool AirSimClient::move_by_velocity_body_frame(float v_forward, float v_right, float v_down, float duration_sec) {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;

    try {
        drone_->MoveByVelocityBodyFrameAsync(
            v_forward, v_right, v_down, duration_sec,
            microsoft::projectairsim::client::YawControlMode::MaxDegreeOfFreedom,
            false, 0.0f, nullptr);
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}

bool AirSimClient::move_by_velocity_yaw(float vx, float vy, float vz, float yaw_rate_rad, float duration_sec) {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;

    try {
        // Project AirSim uses radians for all angles
        drone_->MoveByVelocityAsync(
            vx, vy, vz, duration_sec,
            microsoft::projectairsim::client::YawControlMode::MaxDegreeOfFreedom,
            true,  // yaw_is_rate = true
            yaw_rate_rad,
            nullptr);
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}

bool AirSimClient::apply_velocity_command(const data_types::VelocityCommand& cmd) {
    // Our VelocityCommand uses radians for yaw_rate, matching Project AirSim
    return move_by_velocity_yaw(cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate);
}

bool AirSimClient::move_to_position(float x, float y, float z, float velocity, float timeout_sec) {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;

    try {
        drone_->MoveToPositionAsync(
            x, y, z, velocity, timeout_sec,
            microsoft::projectairsim::client::YawControlMode::MaxDegreeOfFreedom,
            false, 0.0f, nullptr);
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}

bool AirSimClient::set_pose(const Pose& pose, bool reset_kinematics) {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!drone_) return false;

    try {
        microsoft::projectairsim::client::Transform transform;
        transform.position.x = pose.x;
        transform.position.y = pose.y;
        transform.position.z = pose.z;
        transform.orientation.w = pose.qw;
        transform.orientation.x = pose.qx;
        transform.orientation.y = pose.qy;
        transform.orientation.z = pose.qz;

        drone_->SetPose(transform, reset_kinematics);
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}

void AirSimClient::cancel_last_task() {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (drone_) {
        try {
            drone_->CancelLastTask();
        } catch (...) {
            // Ignore
        }
    }
#endif
}

AirSimClient::CollisionInfo AirSimClient::get_collision_info() {
    CollisionInfo info = {false, 0, 0, 0, "", 0};

#ifdef PROJECTAIRSIM_AVAILABLE
    if (!world_) return info;

    try {
        // TODO: Implement when Project AirSim collision API is confirmed
        return info;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return info;
    }
#else
    return info;
#endif
}

bool AirSimClient::reset() {
#ifdef PROJECTAIRSIM_AVAILABLE
    if (!world_) return false;

    try {
        // TODO: Project AirSim reset API
        return true;
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }
#else
    return false;
#endif
}

data_types::ImageData AirSimClient::convert_image(const std::vector<uint8_t>& data,
                                                   int width, int height, int channels) {
    if (data.empty() || width <= 0 || height <= 0) {
        return data_types::ImageData();
    }

    data_types::ImageData img;
    img.width = width;
    img.height = height;
    img.channels = channels;
    img.pixels = data;

    auto now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    img.timestamp_us = now;

    return img;
}

} // namespace sim_interfaces
