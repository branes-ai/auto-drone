#pragma once

/**
 * @file bt_blackboard_sync.hpp
 * @brief Synchronizes Zenoh topics with BehaviorTree blackboard
 *
 * This class subscribes to Zenoh topics and updates the BT blackboard
 * with drone state, target information, and mission status.
 */

#include <behaviortree_cpp/blackboard.h>
#include <ZenohInterface.hpp>
#include <Odometry.hpp>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <cmath>

namespace autodrone::bt {

/**
 * @brief Synchronizes Zenoh state with BehaviorTree blackboard
 *
 * Subscribes to:
 *   - robot/{id}/sensor/state/odom -> drone/pose/*, drone/altitude
 *   - robot/{id}/bt/state/target -> target/*
 *   - robot/{id}/bt/state/inventory -> inventory/*
 *   - robot/{id}/bt/state/mission -> mission/*
 *
 * Also computes derived values like target/distance.
 */
class BlackboardSync {
public:
    /**
     * @brief Construct BlackboardSync
     *
     * @param blackboard BT blackboard to sync state into
     * @param session Zenoh session for subscriptions
     * @param robot_id Robot identifier for topic namespacing
     */
    BlackboardSync(
        BT::Blackboard::Ptr blackboard,
        std::shared_ptr<zenoh_interface::Session> session,
        const std::string& robot_id = "drone"
    );

    ~BlackboardSync();

    // Non-copyable
    BlackboardSync(const BlackboardSync&) = delete;
    BlackboardSync& operator=(const BlackboardSync&) = delete;

    /**
     * @brief Start synchronization (subscribes to topics)
     */
    void start();

    /**
     * @brief Stop synchronization (unsubscribes)
     */
    void stop();

    /**
     * @brief Update computed values (distance, etc.)
     *
     * Call this periodically (e.g., in the main tick loop) to update
     * derived blackboard values that depend on multiple inputs.
     */
    void updateComputed();

    /**
     * @brief Check if odometry has been received
     */
    bool hasOdom() const { return has_odom_.load(); }

    /**
     * @brief Check if target info has been received
     */
    bool hasTarget() const { return has_target_.load(); }

private:
    BT::Blackboard::Ptr blackboard_;
    std::shared_ptr<zenoh_interface::Session> session_;
    std::string robot_id_;

    std::vector<int> subscriber_ids_;
    std::mutex mutex_;

    std::atomic<bool> has_odom_{false};
    std::atomic<bool> has_target_{false};
    std::atomic<bool> running_{false};

    // Cached pose for distance computation
    double drone_x_ = 0.0;
    double drone_y_ = 0.0;
    double drone_z_ = 0.0;
    double target_x_ = 0.0;
    double target_y_ = 0.0;

    void onOdom(const std::string& key, const std::vector<uint8_t>& payload);
    void onTargetState(const std::string& key, const std::vector<uint8_t>& payload);
    void onInventoryState(const std::string& key, const std::vector<uint8_t>& payload);
    void onMissionState(const std::string& key, const std::vector<uint8_t>& payload);

    void parseJson(const std::vector<uint8_t>& payload,
                   const std::function<void(const std::string&)>& handler);
};

} // namespace autodrone::bt
