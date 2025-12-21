#include "behavior_tree/bt_phase_action.hpp"
#include <iostream>
#include <sstream>
#include <cstring>

namespace autodrone::bt {

PhaseAction::PhaseAction(
    const std::string& name,
    const BT::NodeConfig& config,
    std::shared_ptr<zenoh_interface::Session> session,
    const std::string& robot_id
)
    : BT::StatefulActionNode(name, config)
    , session_(std::move(session))
    , robot_id_(robot_id)
{
}

PhaseAction::~PhaseAction() {
    cleanup();
}

BT::NodeStatus PhaseAction::onStart() {
    // Get input parameters
    std::string phase_name;
    if (!getInput("phase_name", phase_name)) {
        std::cerr << "[PhaseAction] Missing required input: phase_name\n";
        return BT::NodeStatus::FAILURE;
    }

    std::string params = "{}";
    getInput("params", params);

    timeout_sec_ = 60.0;
    getInput("timeout_sec", timeout_sec_);

    current_phase_ = phase_name;
    response_received_ = false;
    response_status_.clear();
    response_message_.clear();
    response_data_.clear();

    // Subscribe to response topic
    std::string response_topic = "robot/" + robot_id_ + "/bt/phase/response/" + phase_name;
    response_subscriber_id_ = session_->subscribe(response_topic,
        [this](const std::string& key, const std::vector<uint8_t>& payload) {
            onResponse(key, payload);
        });

    if (response_subscriber_id_ < 0) {
        std::cerr << "[PhaseAction] Failed to subscribe to " << response_topic << "\n";
        return BT::NodeStatus::FAILURE;
    }

    // Record start time
    start_time_ = std::chrono::steady_clock::now();

    // Send request
    sendRequest(phase_name, params);

    std::cout << "[PhaseAction] Started phase '" << phase_name << "'\n";
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PhaseAction::onRunning() {
    // Check for timeout
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - start_time_).count();

    if (elapsed > timeout_sec_) {
        std::cerr << "[PhaseAction] Phase '" << current_phase_ << "' timed out after "
                  << elapsed << "s\n";
        setOutput("result_status", std::string("TIMEOUT"));
        setOutput("result_message", std::string("Phase execution timed out"));
        setOutput("result_data", std::string("{}"));
        cleanup();
        return BT::NodeStatus::FAILURE;
    }

    // Check if response received
    if (response_received_.load()) {
        std::lock_guard<std::mutex> lock(response_mutex_);

        setOutput("result_status", response_status_);
        setOutput("result_message", response_message_);
        setOutput("result_data", response_data_);

        cleanup();

        if (response_status_ == "COMPLETED") {
            std::cout << "[PhaseAction] Phase '" << current_phase_ << "' completed: "
                      << response_message_ << "\n";
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cerr << "[PhaseAction] Phase '" << current_phase_ << "' failed: "
                      << response_message_ << "\n";
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void PhaseAction::onHalted() {
    std::cout << "[PhaseAction] Phase '" << current_phase_ << "' halted\n";
    sendCancel(current_phase_);
    cleanup();
}

void PhaseAction::sendRequest(const std::string& phase_name, const std::string& params) {
    std::string request_topic = "robot/" + robot_id_ + "/bt/phase/request/" + phase_name;

    // Build JSON request
    std::ostringstream json;
    json << "{\"params\": " << params << "}";
    std::string json_str = json.str();

    std::vector<uint8_t> payload(json_str.begin(), json_str.end());
    session_->publish(request_topic, payload);

    std::cout << "[PhaseAction] Sent request to " << request_topic << "\n";
}

void PhaseAction::sendCancel(const std::string& phase_name) {
    std::string cancel_topic = "robot/" + robot_id_ + "/bt/phase/cancel/" + phase_name;

    std::string json_str = "{\"cancel\": true}";
    std::vector<uint8_t> payload(json_str.begin(), json_str.end());
    session_->publish(cancel_topic, payload);
}

void PhaseAction::onResponse(const std::string& /*key*/, const std::vector<uint8_t>& payload) {
    if (payload.empty()) return;

    std::string json(payload.begin(), payload.end());

    // Parse response JSON: {"status": str, "message": str, "data": {...}}
    std::lock_guard<std::mutex> lock(response_mutex_);

    // Parse status
    auto parse_string = [&json](const std::string& key) -> std::string {
        std::string pattern = "\"" + key + "\":";
        size_t pos = json.find(pattern);
        if (pos == std::string::npos) {
            pattern = "\"" + key + "\": ";
            pos = json.find(pattern);
        }
        if (pos != std::string::npos) {
            pos += pattern.length();
            // Skip whitespace
            while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
            if (pos < json.length() && json[pos] == '"') {
                size_t start = pos + 1;
                size_t end = json.find('"', start);
                if (end != std::string::npos) {
                    return json.substr(start, end - start);
                }
            }
        }
        return "";
    };

    response_status_ = parse_string("status");
    response_message_ = parse_string("message");

    // Extract data object (everything between "data": and the next top-level comma/brace)
    size_t data_pos = json.find("\"data\":");
    if (data_pos == std::string::npos) {
        data_pos = json.find("\"data\": ");
    }
    if (data_pos != std::string::npos) {
        size_t start = json.find('{', data_pos);
        if (start != std::string::npos) {
            int brace_count = 1;
            size_t end = start + 1;
            while (end < json.length() && brace_count > 0) {
                if (json[end] == '{') brace_count++;
                else if (json[end] == '}') brace_count--;
                end++;
            }
            response_data_ = json.substr(start, end - start);
        }
    } else {
        response_data_ = "{}";
    }

    response_received_ = true;
}

void PhaseAction::cleanup() {
    if (response_subscriber_id_ >= 0) {
        session_->unsubscribe(response_subscriber_id_);
        response_subscriber_id_ = -1;
    }
    current_phase_.clear();
}

} // namespace autodrone::bt
