#pragma once

#include "ImageData.hpp"
#include "Odometry.hpp"
#include "VelocityCommand.hpp"

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <chrono>

// Forward declarations for Project AirSim types
// Project AirSim uses microsoft::projectairsim::client namespace
namespace microsoft { namespace projectairsim { namespace client {
    class Client;
    class World;
    class Drone;
}}}

namespace sim_interfaces {

// Camera types available in Project AirSim
enum class CameraType {
    Scene = 0,              // RGB Scene
    DepthPlanar = 1,        // Depth planar
    DepthPerspective = 2,   // Depth perspective
    Segmentation = 3,       // Segmentation
    DepthVis = 4,           // Depth visualization
    DisparityNormalized = 5,
    SurfaceNormals = 6
};

// Connection configuration for Project AirSim
struct AirSimConfig {
    std::string host = "localhost";
    int port = 41451;
    std::string vehicle_name = "Drone";
    float timeout_sec = 60.0f;

    // Camera settings
    std::string camera_name = "front_center";
    int image_width = 640;
    int image_height = 480;
    bool capture_rgb = true;
    bool capture_depth = false;

    // Control settings
    bool enable_api_control = true;
};

// Project AirSim multirotor client wrapper
// Provides high-level interface for:
// - Connecting to Project AirSim
// - Capturing camera images
// - Reading drone state (pose, velocity)
// - Sending velocity commands
//
// NOTE: This is for Project AirSim (IAMAI), not the old Microsoft AirSim.
// Project AirSim uses UE5 and has a different API structure.
// See: https://github.com/iamaisim/ProjectAirSim
class AirSimClient {
public:
    AirSimClient();
    explicit AirSimClient(const AirSimConfig& config);
    ~AirSimClient();

    // Non-copyable, movable
    AirSimClient(const AirSimClient&) = delete;
    AirSimClient& operator=(const AirSimClient&) = delete;
    AirSimClient(AirSimClient&&) noexcept;
    AirSimClient& operator=(AirSimClient&&) noexcept;

    // Connection management
    bool connect();
    void disconnect();
    bool is_connected() const;

    // Configuration
    void set_config(const AirSimConfig& config);
    const AirSimConfig& config() const { return config_; }

    // API control (required for sending commands)
    bool enable_api_control();
    bool disable_api_control();
    bool is_api_control_enabled() const;

    // Arming (Project AirSim uses separate arm/disarm methods)
    bool arm();
    bool disarm();
    bool can_arm() const;

    // Takeoff / Land
    bool takeoff(float timeout_sec = 20.0f);
    bool land(float timeout_sec = 60.0f);
    bool hover();
    bool go_home();

    // Image capture
    // Returns empty ImageData if capture fails
    data_types::ImageData capture_rgb();
    data_types::ImageData capture_depth();
    std::vector<data_types::ImageData> capture_all();

    // State queries
    data_types::Odometry get_odometry();

    // Get raw pose (position + orientation quaternion)
    struct Pose {
        float x, y, z;           // Position (NED frame, meters)
        float qw, qx, qy, qz;    // Orientation quaternion
    };
    Pose get_pose();

    // Get velocity
    struct Velocity {
        float vx, vy, vz;        // Linear velocity (NED frame, m/s)
        float wx, wy, wz;        // Angular velocity (rad/s)
    };
    Velocity get_velocity();

    // Movement commands
    // All velocities in NED frame (North-East-Down)
    // All angles in RADIANS (Project AirSim standard)
    // Duration: how long to apply the command (0 = until next command)
    bool move_by_velocity(float vx, float vy, float vz, float duration_sec = 0.1f);
    bool move_by_velocity_body_frame(float v_forward, float v_right, float v_down, float duration_sec = 0.1f);
    bool move_by_velocity_yaw(float vx, float vy, float vz, float yaw_rate_rad, float duration_sec = 0.1f);

    // Apply velocity command from our data type
    bool apply_velocity_command(const data_types::VelocityCommand& cmd);

    // Move to position
    bool move_to_position(float x, float y, float z, float velocity = 1.0f, float timeout_sec = 60.0f);

    // Set pose directly (useful for teleporting in simulation)
    bool set_pose(const Pose& pose, bool reset_kinematics = true);

    // Cancel current async task
    void cancel_last_task();

    // Get collision info
    struct CollisionInfo {
        bool has_collided;
        float impact_x, impact_y, impact_z;
        std::string object_name;
        uint64_t time_stamp;
    };
    CollisionInfo get_collision_info();

    // Reset simulation
    bool reset();

    // Get last error message
    std::string last_error() const { return last_error_; }

private:
    AirSimConfig config_;

    // Project AirSim uses separate Client, World, and Drone objects
    std::shared_ptr<microsoft::projectairsim::client::Client> client_;
    std::shared_ptr<microsoft::projectairsim::client::World> world_;
    std::shared_ptr<microsoft::projectairsim::client::Drone> drone_;

    bool connected_ = false;
    bool api_control_enabled_ = false;
    std::string last_error_;

    // Helper to convert Project AirSim image to our format
    data_types::ImageData convert_image(const std::vector<uint8_t>& data,
                                         int width, int height, int channels);
};

} // namespace sim_interfaces
