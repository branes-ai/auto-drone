#pragma once

/**
 * @file bt_phase_action.hpp
 * @brief PhaseAction node for executing Python mission phases via Zenoh RPC
 *
 * This action node sends phase execution requests to the Python phase server
 * and waits for completion, enabling behavior trees to orchestrate existing
 * mission phases.
 */

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ZenohInterface.hpp>
#include <string>
#include <chrono>
#include <atomic>
#include <mutex>
#include <condition_variable>

namespace autodrone::bt {

/**
 * @brief Execute a Python mission phase via Zenoh RPC
 *
 * This is a StatefulActionNode that:
 * 1. onStart(): Sends a phase request to Python phase server
 * 2. onRunning(): Polls for response (non-blocking)
 * 3. onHalted(): Sends cancellation request
 *
 * Input Ports:
 *   - phase_name: Name of the phase to execute (e.g., "ascend", "scan")
 *   - params: JSON string of phase parameters (default: "{}")
 *   - timeout_sec: Execution timeout in seconds (default: 60)
 *
 * Output Ports:
 *   - result_status: "COMPLETED", "FAILED", or "TIMEOUT"
 *   - result_data: JSON string of phase output data
 *   - result_message: Human-readable result message
 *
 * Zenoh Topics:
 *   - PUT robot/{id}/bt/phase/request/{phase_name} - Send request
 *   - SUB robot/{id}/bt/phase/response/{phase_name} - Receive response
 */
class PhaseAction : public BT::StatefulActionNode {
public:
    /**
     * @brief Construct PhaseAction
     *
     * @param name Node name
     * @param config Node configuration
     * @param session Zenoh session for RPC
     * @param robot_id Robot identifier for topic namespacing
     */
    PhaseAction(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<zenoh_interface::Session> session,
        const std::string& robot_id
    );

    ~PhaseAction() override;

    static BT::PortsList providedPorts() {
        return {
            // Inputs
            BT::InputPort<std::string>("phase_name", "Name of the phase to execute"),
            BT::InputPort<std::string>("params", "{}", "JSON parameters for the phase"),
            BT::InputPort<double>("timeout_sec", 60.0, "Timeout in seconds"),

            // Outputs
            BT::OutputPort<std::string>("result_status", "COMPLETED, FAILED, or TIMEOUT"),
            BT::OutputPort<std::string>("result_data", "JSON output from phase"),
            BT::OutputPort<std::string>("result_message", "Result message")
        };
    }

    /**
     * @brief Called when the node transitions to RUNNING
     *
     * Sends the phase request to Python phase server.
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Called while the node is RUNNING
     *
     * Checks for response from Python phase server.
     * Returns RUNNING if still waiting, SUCCESS/FAILURE when complete.
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Called when the node is halted (e.g., by parent Sequence failing)
     *
     * Sends cancellation request to Python phase server.
     */
    void onHalted() override;

private:
    std::shared_ptr<zenoh_interface::Session> session_;
    std::string robot_id_;

    // Current execution state
    std::string current_phase_;
    std::chrono::steady_clock::time_point start_time_;
    double timeout_sec_ = 60.0;
    int response_subscriber_id_ = -1;

    // Response handling
    std::atomic<bool> response_received_{false};
    std::mutex response_mutex_;
    std::string response_status_;
    std::string response_message_;
    std::string response_data_;

    void sendRequest(const std::string& phase_name, const std::string& params);
    void sendCancel(const std::string& phase_name);
    void onResponse(const std::string& key, const std::vector<uint8_t>& payload);
    void cleanup();
};

} // namespace autodrone::bt
