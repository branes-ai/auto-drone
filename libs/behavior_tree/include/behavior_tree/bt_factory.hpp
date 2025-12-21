#pragma once

/**
 * @file bt_factory.hpp
 * @brief Factory setup and node registration for BehaviorTree.CPP integration
 *
 * This module provides factory configuration for the auto-drone behavior tree
 * system, registering all custom nodes (conditions, actions) with the BT factory.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <ZenohInterface.hpp>
#include <memory>
#include <string>

namespace autodrone::bt {

/**
 * @brief Configuration for the behavior tree factory
 */
struct FactoryConfig {
    std::shared_ptr<zenoh_interface::Session> session;
    std::string robot_id = "drone";

    FactoryConfig() = default;
    FactoryConfig(std::shared_ptr<zenoh_interface::Session> sess, const std::string& id = "drone")
        : session(std::move(sess)), robot_id(id) {}
};

/**
 * @brief Register all custom nodes with the BT factory
 *
 * Registers:
 *   - Condition nodes: HasTarget, HasDetections, CheckAltitude, CheckDistance, IsMissionRunning
 *   - Action nodes: PhaseAction
 *
 * @param factory The BehaviorTree factory to register nodes with
 * @param config Configuration including Zenoh session and robot ID
 */
void registerAllNodes(BT::BehaviorTreeFactory& factory, const FactoryConfig& config);

/**
 * @brief Register only condition nodes (for testing without Zenoh)
 *
 * @param factory The BehaviorTree factory to register nodes with
 */
void registerConditionNodes(BT::BehaviorTreeFactory& factory);

} // namespace autodrone::bt
