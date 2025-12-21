#include "behavior_tree/bt_factory.hpp"
#include "behavior_tree/bt_conditions.hpp"
#include "behavior_tree/bt_phase_action.hpp"

namespace autodrone::bt {

void registerConditionNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<HasTarget>("HasTarget");
    factory.registerNodeType<HasDetections>("HasDetections");
    factory.registerNodeType<CheckAltitude>("CheckAltitude");
    factory.registerNodeType<CheckDistance>("CheckDistance");
    factory.registerNodeType<CheckMinAltitude>("CheckMinAltitude");
    factory.registerNodeType<IsMissionRunning>("IsMissionRunning");
}

void registerAllNodes(BT::BehaviorTreeFactory& factory, const FactoryConfig& config) {
    // Register condition nodes
    registerConditionNodes(factory);

    // Register PhaseAction with Zenoh session dependency
    // Using a lambda to capture the session and robot_id
    auto phase_action_builder = [session = config.session, robot_id = config.robot_id](
        const std::string& name, const BT::NodeConfig& node_config
    ) {
        return std::make_unique<PhaseAction>(name, node_config, session, robot_id);
    };

    factory.registerBuilder<PhaseAction>("PhaseAction", phase_action_builder);
}

} // namespace autodrone::bt
