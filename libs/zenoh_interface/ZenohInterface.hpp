#pragma once

#include <zenoh.h>
#include <string>
#include <vector>
#include <functional>
#include <memory>

namespace zenoh_interface {

// Define the function signature for a generic Zenoh callback
using ZenohCallback = std::function<void(const std::string& key, const std::vector<uint8_t>& payload)>;

// Forward declaration for subscriber context
struct SubscriberContext;

// Configuration for Zenoh session
struct SessionConfig {
    // Remote endpoints to connect to (e.g., "tcp/192.168.1.10:7447")
    std::vector<std::string> connect_endpoints;

    // Local endpoints to listen on (e.g., "tcp/0.0.0.0:7447")
    std::vector<std::string> listen_endpoints;

    // Scouting mode: "peer", "router", or "none" (empty = use Zenoh default)
    std::string mode;

    // Default configuration (local scouting with Zenoh defaults)
    static SessionConfig local() { return SessionConfig{}; }

    // Connect to a remote endpoint
    static SessionConfig connect_to(const std::string& endpoint) {
        SessionConfig cfg;
        cfg.connect_endpoints.push_back(endpoint);
        return cfg;
    }
};

// RAII wrapper for Zenoh session
class Session {
public:
    // Default constructor - uses local scouting
    Session();

    // Constructor with configuration
    explicit Session(const SessionConfig& config);

    ~Session();

    // Non-copyable
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    // Movable
    Session(Session&& other) noexcept;
    Session& operator=(Session&& other) noexcept;

    bool is_valid() const;

    // Publish data to a key expression
    bool publish(const std::string& key_expr, const std::vector<uint8_t>& payload);

    // Subscribe to a key expression with a callback
    // Returns subscriber ID, or -1 on failure
    int subscribe(const std::string& key_expr, ZenohCallback callback);

    // Unsubscribe by ID
    void unsubscribe(int subscriber_id);

private:
    z_owned_session_t session_;
    bool valid_ = false;
    std::vector<std::unique_ptr<SubscriberContext>> subscribers_;
};

} // namespace zenoh_interface
