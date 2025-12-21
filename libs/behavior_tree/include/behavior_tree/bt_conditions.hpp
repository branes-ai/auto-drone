#pragma once

/**
 * @file bt_conditions.hpp
 * @brief Condition nodes for behavior tree decision-making
 *
 * These nodes check blackboard state to make branching decisions.
 * All conditions are synchronous and return SUCCESS or FAILURE.
 */

#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <cmath>

namespace autodrone::bt {

/**
 * @brief Check if a target has been selected
 *
 * Reads: target/selected (bool)
 * Returns SUCCESS if target is selected, FAILURE otherwise
 */
class HasTarget : public BT::ConditionNode {
public:
    HasTarget(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override {
        auto bb = config().blackboard;
        auto selected = bb->get<bool>("target/selected");
        return selected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

/**
 * @brief Check if objects have been detected
 *
 * Reads: inventory/target_count (int)
 * Input ports:
 *   - min_count: Minimum number of detections required (default: 1)
 *
 * Returns SUCCESS if count >= min_count, FAILURE otherwise
 */
class HasDetections : public BT::ConditionNode {
public:
    HasDetections(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("min_count", 1, "Minimum detection count required")
        };
    }

    BT::NodeStatus tick() override {
        int min_count = 1;
        getInput("min_count", min_count);

        auto bb = config().blackboard;
        int count = 0;
        try {
            count = bb->get<int>("inventory/target_count");
        } catch (...) {
            // Key not set yet
            count = 0;
        }

        return (count >= min_count) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

/**
 * @brief Check if drone is at target altitude
 *
 * Reads: drone/altitude (double) - positive meters above ground
 * Input ports:
 *   - target_altitude: Target altitude in meters
 *   - tolerance: Altitude tolerance in meters (default: 0.5)
 *
 * Returns SUCCESS if within tolerance, FAILURE otherwise
 */
class CheckAltitude : public BT::ConditionNode {
public:
    CheckAltitude(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("target_altitude", "Target altitude in meters"),
            BT::InputPort<double>("tolerance", 0.5, "Altitude tolerance in meters")
        };
    }

    BT::NodeStatus tick() override {
        double target_alt = 0.0;
        double tolerance = 0.5;

        if (!getInput("target_altitude", target_alt)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("tolerance", tolerance);

        auto bb = config().blackboard;
        double current_alt = 0.0;
        try {
            current_alt = bb->get<double>("drone/altitude");
        } catch (...) {
            return BT::NodeStatus::FAILURE;
        }

        double error = std::abs(current_alt - target_alt);
        return (error <= tolerance) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

/**
 * @brief Check if drone is within distance threshold of target
 *
 * Reads: target/distance (double) - meters to target
 * Input ports:
 *   - threshold: Distance threshold in meters
 *
 * Returns SUCCESS if distance <= threshold, FAILURE otherwise
 */
class CheckDistance : public BT::ConditionNode {
public:
    CheckDistance(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("threshold", "Distance threshold in meters")
        };
    }

    BT::NodeStatus tick() override {
        double threshold = 0.0;
        if (!getInput("threshold", threshold)) {
            return BT::NodeStatus::FAILURE;
        }

        auto bb = config().blackboard;
        double distance = std::numeric_limits<double>::max();
        try {
            distance = bb->get<double>("target/distance");
        } catch (...) {
            return BT::NodeStatus::FAILURE;
        }

        return (distance <= threshold) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

/**
 * @brief Check if mission is currently running
 *
 * Reads: mission/running (bool)
 * Returns SUCCESS if running, FAILURE otherwise
 */
class IsMissionRunning : public BT::ConditionNode {
public:
    IsMissionRunning(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override {
        auto bb = config().blackboard;
        bool running = false;
        try {
            running = bb->get<bool>("mission/running");
        } catch (...) {
            running = false;
        }
        return running ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

/**
 * @brief Check if altitude is above minimum threshold
 *
 * Reads: drone/altitude (double)
 * Input ports:
 *   - min_altitude: Minimum altitude in meters
 *
 * Returns SUCCESS if altitude >= min_altitude, FAILURE otherwise
 */
class CheckMinAltitude : public BT::ConditionNode {
public:
    CheckMinAltitude(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("min_altitude", "Minimum altitude in meters")
        };
    }

    BT::NodeStatus tick() override {
        double min_alt = 0.0;
        if (!getInput("min_altitude", min_alt)) {
            return BT::NodeStatus::FAILURE;
        }

        auto bb = config().blackboard;
        double current_alt = 0.0;
        try {
            current_alt = bb->get<double>("drone/altitude");
        } catch (...) {
            return BT::NodeStatus::FAILURE;
        }

        return (current_alt >= min_alt) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

} // namespace autodrone::bt
