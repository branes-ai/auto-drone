#include "behavior_tree/bt_blackboard_sync.hpp"
#include <iostream>
#include <sstream>
#include <cstring>

namespace autodrone::bt {

BlackboardSync::BlackboardSync(
    BT::Blackboard::Ptr blackboard,
    std::shared_ptr<zenoh_interface::Session> session,
    const std::string& robot_id
)
    : blackboard_(std::move(blackboard))
    , session_(std::move(session))
    , robot_id_(robot_id)
{
}

BlackboardSync::~BlackboardSync() {
    stop();
}

void BlackboardSync::start() {
    if (running_.load()) return;
    running_ = true;

    // Subscribe to odometry
    std::string odom_topic = "robot/" + robot_id_ + "/sensor/state/odom";
    int odom_sub = session_->subscribe(odom_topic,
        [this](const std::string& key, const std::vector<uint8_t>& payload) {
            onOdom(key, payload);
        });
    if (odom_sub >= 0) {
        subscriber_ids_.push_back(odom_sub);
        std::cout << "[BlackboardSync] Subscribed to " << odom_topic << "\n";
    }

    // Subscribe to target state from Python
    std::string target_topic = "robot/" + robot_id_ + "/bt/state/target";
    int target_sub = session_->subscribe(target_topic,
        [this](const std::string& key, const std::vector<uint8_t>& payload) {
            onTargetState(key, payload);
        });
    if (target_sub >= 0) {
        subscriber_ids_.push_back(target_sub);
        std::cout << "[BlackboardSync] Subscribed to " << target_topic << "\n";
    }

    // Subscribe to inventory state from Python
    std::string inventory_topic = "robot/" + robot_id_ + "/bt/state/inventory";
    int inv_sub = session_->subscribe(inventory_topic,
        [this](const std::string& key, const std::vector<uint8_t>& payload) {
            onInventoryState(key, payload);
        });
    if (inv_sub >= 0) {
        subscriber_ids_.push_back(inv_sub);
        std::cout << "[BlackboardSync] Subscribed to " << inventory_topic << "\n";
    }

    // Subscribe to mission state from Python
    std::string mission_topic = "robot/" + robot_id_ + "/bt/state/mission";
    int mission_sub = session_->subscribe(mission_topic,
        [this](const std::string& key, const std::vector<uint8_t>& payload) {
            onMissionState(key, payload);
        });
    if (mission_sub >= 0) {
        subscriber_ids_.push_back(mission_sub);
        std::cout << "[BlackboardSync] Subscribed to " << mission_topic << "\n";
    }

    // Initialize blackboard with defaults
    blackboard_->set("target/selected", false);
    blackboard_->set("target/distance", std::numeric_limits<double>::max());
    blackboard_->set("inventory/count", 0);
    blackboard_->set("inventory/target_count", 0);
    blackboard_->set("mission/running", false);
}

void BlackboardSync::stop() {
    if (!running_.load()) return;
    running_ = false;

    for (int sub_id : subscriber_ids_) {
        session_->unsubscribe(sub_id);
    }
    subscriber_ids_.clear();
}

void BlackboardSync::updateComputed() {
    std::lock_guard<std::mutex> lock(mutex_);

    // Compute distance to target if both positions are known
    if (has_odom_.load() && has_target_.load()) {
        double dx = target_x_ - drone_x_;
        double dy = target_y_ - drone_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        blackboard_->set("target/distance", distance);
    }
}

void BlackboardSync::onOdom(const std::string& /*key*/, const std::vector<uint8_t>& payload) {
    // Deserialize Odometry (binary format from data_types)
    if (payload.size() < sizeof(data_types::Odometry)) {
        return;
    }

    data_types::Odometry odom;
    std::memcpy(&odom, payload.data(), sizeof(data_types::Odometry));

    {
        std::lock_guard<std::mutex> lock(mutex_);
        drone_x_ = odom.x;
        drone_y_ = odom.y;
        drone_z_ = odom.z;
    }

    // Update blackboard (NED coordinates: z is negative for altitude)
    blackboard_->set("drone/pose/x", static_cast<double>(odom.x));
    blackboard_->set("drone/pose/y", static_cast<double>(odom.y));
    blackboard_->set("drone/pose/z", static_cast<double>(odom.z));
    blackboard_->set("drone/pose/yaw", static_cast<double>(odom.yaw));
    blackboard_->set("drone/altitude", static_cast<double>(-odom.z));  // Convert NED to positive altitude

    has_odom_ = true;
}

void BlackboardSync::onTargetState(const std::string& /*key*/, const std::vector<uint8_t>& payload) {
    // Parse JSON: {"selected": bool, "x": float, "y": float, "class": str}
    parseJson(payload, [this](const std::string& json) {
        // Simple JSON parsing (no external dependency)
        bool selected = json.find("\"selected\": true") != std::string::npos ||
                        json.find("\"selected\":true") != std::string::npos;
        blackboard_->set("target/selected", selected);

        // Parse x coordinate
        auto parse_double = [&json](const std::string& key) -> double {
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
                size_t end = json.find_first_of(",}", pos);
                if (end != std::string::npos) {
                    try {
                        return std::stod(json.substr(pos, end - pos));
                    } catch (...) {}
                }
            }
            return 0.0;
        };

        if (selected) {
            double x = parse_double("x");
            double y = parse_double("y");

            {
                std::lock_guard<std::mutex> lock(mutex_);
                target_x_ = x;
                target_y_ = y;
            }

            blackboard_->set("target/x", x);
            blackboard_->set("target/y", y);
            has_target_ = true;
        } else {
            has_target_ = false;
        }

        // Parse class name
        size_t class_pos = json.find("\"class\":");
        if (class_pos == std::string::npos) {
            class_pos = json.find("\"class\": ");
        }
        if (class_pos != std::string::npos) {
            size_t quote_start = json.find('"', class_pos + 7);
            if (quote_start != std::string::npos) {
                size_t quote_end = json.find('"', quote_start + 1);
                if (quote_end != std::string::npos) {
                    std::string class_name = json.substr(quote_start + 1, quote_end - quote_start - 1);
                    blackboard_->set("target/class", class_name);
                }
            }
        }
    });
}

void BlackboardSync::onInventoryState(const std::string& /*key*/, const std::vector<uint8_t>& payload) {
    // Parse JSON: {"count": int, "target_count": int}
    parseJson(payload, [this](const std::string& json) {
        auto parse_int = [&json](const std::string& key) -> int {
            std::string pattern = "\"" + key + "\":";
            size_t pos = json.find(pattern);
            if (pos == std::string::npos) {
                pattern = "\"" + key + "\": ";
                pos = json.find(pattern);
            }
            if (pos != std::string::npos) {
                pos += pattern.length();
                while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
                size_t end = json.find_first_of(",}", pos);
                if (end != std::string::npos) {
                    try {
                        return std::stoi(json.substr(pos, end - pos));
                    } catch (...) {}
                }
            }
            return 0;
        };

        int count = parse_int("count");
        int target_count = parse_int("target_count");

        blackboard_->set("inventory/count", count);
        blackboard_->set("inventory/target_count", target_count);
    });
}

void BlackboardSync::onMissionState(const std::string& /*key*/, const std::vector<uint8_t>& payload) {
    // Parse JSON: {"running": bool}
    parseJson(payload, [this](const std::string& json) {
        bool running = json.find("\"running\": true") != std::string::npos ||
                       json.find("\"running\":true") != std::string::npos;
        blackboard_->set("mission/running", running);
    });
}

void BlackboardSync::parseJson(const std::vector<uint8_t>& payload,
                               const std::function<void(const std::string&)>& handler) {
    if (payload.empty()) return;

    std::string json(payload.begin(), payload.end());
    handler(json);
}

} // namespace autodrone::bt
