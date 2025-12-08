#include "ZenohInterface.hpp"
#include <iostream>
#include <cstring>

namespace zenoh_interface {

// Context for subscriber callback - holds the user's callback function
struct SubscriberContext {
    ZenohCallback callback;
    z_owned_subscriber_t subscriber;
    bool active = false;
};

// C callback that bridges to C++ std::function
static void sample_handler(z_loaned_sample_t* sample, void* arg) {
    auto* ctx = static_cast<SubscriberContext*>(arg);
    if (!ctx || !ctx->active) return;

    // Extract key expression
    z_view_string_t key_view;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &key_view);
    std::string key(z_string_data(z_loan(key_view)), z_string_len(z_loan(key_view)));

    // Extract payload
    const z_loaned_bytes_t* payload_bytes = z_sample_payload(sample);
    z_bytes_reader_t reader = z_bytes_get_reader(payload_bytes);
    size_t payload_len = z_bytes_len(payload_bytes);

    std::vector<uint8_t> payload(payload_len);
    if (payload_len > 0) {
        z_bytes_reader_read(&reader, payload.data(), payload_len);
    }

    // Invoke user callback
    ctx->callback(key, payload);
}

Session::Session() {
    z_owned_config_t config;
    z_config_default(&config);

    if (z_open(&session_, z_move(config), nullptr) < 0) {
        std::cerr << "Error opening Zenoh session." << std::endl;
        valid_ = false;
    } else {
        std::cout << "Zenoh session opened successfully." << std::endl;
        valid_ = true;
    }
}

Session::~Session() {
    // Clean up subscribers first
    for (auto& ctx : subscribers_) {
        if (ctx && ctx->active) {
            ctx->active = false;
            z_drop(z_move(ctx->subscriber));
        }
    }
    subscribers_.clear();

    // Close session
    if (valid_) {
        z_drop(z_move(session_));
    }
}

Session::Session(Session&& other) noexcept
    : session_(other.session_)
    , valid_(other.valid_)
    , subscribers_(std::move(other.subscribers_)) {
    other.valid_ = false;
}

Session& Session::operator=(Session&& other) noexcept {
    if (this != &other) {
        // Clean up current resources
        for (auto& ctx : subscribers_) {
            if (ctx && ctx->active) {
                ctx->active = false;
                z_drop(z_move(ctx->subscriber));
            }
        }
        if (valid_) {
            z_drop(z_move(session_));
        }

        // Move from other
        session_ = other.session_;
        valid_ = other.valid_;
        subscribers_ = std::move(other.subscribers_);
        other.valid_ = false;
    }
    return *this;
}

bool Session::is_valid() const {
    return valid_;
}

bool Session::publish(const std::string& key_expr, const std::vector<uint8_t>& payload) {
    if (!valid_) {
        std::cerr << "Cannot publish: session is not valid." << std::endl;
        return false;
    }

    // Create key expression
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, key_expr.c_str()) < 0) {
        std::cerr << "Invalid key expression: " << key_expr << std::endl;
        return false;
    }

    // Create payload bytes
    z_owned_bytes_t bytes;
    z_bytes_from_buf(&bytes, payload.data(), payload.size(), nullptr, nullptr);

    // Publish
    z_put_options_t options;
    z_put_options_default(&options);

    int result = z_put(z_loan(session_), z_loan(ke), z_move(bytes), &options);

    if (result < 0) {
        std::cerr << "Failed to publish to " << key_expr << std::endl;
        return false;
    }

    return true;
}

int Session::subscribe(const std::string& key_expr, ZenohCallback callback) {
    if (!valid_) {
        std::cerr << "Cannot subscribe: session is not valid." << std::endl;
        return -1;
    }

    // Create subscriber context
    auto ctx = std::make_unique<SubscriberContext>();
    ctx->callback = std::move(callback);

    // Create key expression
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, key_expr.c_str()) < 0) {
        std::cerr << "Invalid key expression: " << key_expr << std::endl;
        return -1;
    }

    // Create closure with callback
    z_owned_closure_sample_t closure;
    z_closure(&closure, sample_handler, nullptr, ctx.get());

    // Declare subscriber
    if (z_declare_subscriber(z_loan(session_), &ctx->subscriber, z_loan(ke), z_move(closure), nullptr) < 0) {
        std::cerr << "Failed to subscribe to " << key_expr << std::endl;
        return -1;
    }

    ctx->active = true;
    int id = static_cast<int>(subscribers_.size());
    subscribers_.push_back(std::move(ctx));

    std::cout << "Subscribed to " << key_expr << " (id=" << id << ")" << std::endl;
    return id;
}

void Session::unsubscribe(int subscriber_id) {
    if (subscriber_id < 0 || subscriber_id >= static_cast<int>(subscribers_.size())) {
        return;
    }

    auto& ctx = subscribers_[subscriber_id];
    if (ctx && ctx->active) {
        ctx->active = false;
        z_drop(z_move(ctx->subscriber));
        std::cout << "Unsubscribed (id=" << subscriber_id << ")" << std::endl;
    }
}

} // namespace zenoh_interface
