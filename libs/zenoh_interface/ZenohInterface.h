#pragma once

#include <zenohc.h>
#include <string>
#include <vector>
#include <functional>

namespace zenoh_interface {

// Define the function signature for a generic Zenoh callback
using ZenohCallback = std::function<void(const std::string& key, const std::vector<uint8_t>& payload)>;

/**
 * @brief Initializes and opens a Zenoh session.
 * @return A valid zenohc_session_t pointer or nullptr on failure.
 */
zenohc_session_t* init_session();

/**
 * @brief Closes and releases the Zenoh session resources.
 * @param session The session to close.
 */
void close_session(zenohc_session_t* session);

/**
 * @brief Publishes data to a specific Zenoh key expression.
 * @param session The active Zenoh session.
 * @param key_expr The Zenoh key expression (e.g., "robot/drone/sensor/camera/rgb").
 * @param payload The raw byte data to publish.
 * @return true on success, false otherwise.
 */
bool publish(zenohc_session_t* session, const std::string& key_expr, const std::vector<uint8_t>& payload);

/**
 * @brief Subscribes to a key expression and registers a callback function.
 * @param session The active Zenoh session.
 * @param key_expr The key expression to subscribe to (can include wildcards).
 * @param callback The function to execute when a new sample arrives.
 * @return A zenohc_subscriber_t pointer for managing the subscription.
 */
zenohc_subscriber_t* subscribe(zenohc_session_t* session, const std::string& key_expr, ZenohCallback callback);

} // namespace zenoh_interface