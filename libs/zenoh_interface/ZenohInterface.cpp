#include "ZenohInterface.h"
#include <iostream>

namespace zenoh_interface {

// --- Internal Zenoh-C Callback Handler ---
void sample_handler(const zenohc_sample_t* sample, void* arg) {
    ZenohCallback* user_callback = static_cast<ZenohCallback*>(arg);
    
    // Extract key and payload
    std::string key(zenohc_sample_key(sample), zenohc_sample_keylen(sample));
    
    const uint8_t* data = zenohc_sample_payload(sample);
    size_t len = zenohc_sample_payloadlen(sample);

    // Copy payload into a C++ vector
    std::vector<uint8_t> payload(data, data + len);

    // Execute the user's C++ callback function
    (*user_callback)(key, payload);
}

zenohc_session_t* init_session() {
    // 1. Configure Zenoh (e.g., peer mode, router IP, properties)
    zenohc_config_t* config = zenohc_config_new();
    // Add configuration logic here (e.g., config = zenohc_config_from_file("zenoh.json"))
    
    // 2. Open a session
    zenohc_session_t* session = zenohc_open(config);
    zenohc_config_delete(config);

    if (session == nullptr) {
        std::cerr << "Error opening Zenoh session." << std::endl;
    } else {
        std::cout << "Zenoh session opened successfully." << std::endl;
    }
    return session;
}

// ... implementation for publish() and subscribe() using sample_handler ...

} // namespace zenoh_interface