#include <catch2/catch_test_macros.hpp>
#include "ZenohInterface.hpp"
#include <thread>
#include <chrono>
#include <atomic>

using namespace zenoh_interface;

// Note: These tests require a running Zenoh router or peer-to-peer connectivity.
// They are integration tests that verify actual Zenoh functionality.

TEST_CASE("Session creation and destruction", "[Session][integration]") {
    Session session;

    // Session should be valid after construction (assuming Zenoh is available)
    // If Zenoh daemon is not running, this may fail
    if (session.is_valid()) {
        SUCCEED("Session created successfully");
    } else {
        WARN("Session creation failed - is Zenoh router running?");
    }
}

TEST_CASE("Publish returns false on invalid session", "[Session]") {
    // Create a session, then move from it to invalidate
    Session session;
    Session moved_session = std::move(session);

    // Original session should now be invalid
    REQUIRE_FALSE(session.is_valid());

    std::vector<uint8_t> payload = {1, 2, 3};
    REQUIRE_FALSE(session.publish("test/key", payload));
}

TEST_CASE("Subscribe returns -1 on invalid session", "[Session]") {
    Session session;
    Session moved_session = std::move(session);

    REQUIRE_FALSE(session.is_valid());
    int id = session.subscribe("test/key", [](const std::string&, const std::vector<uint8_t>&) {});
    REQUIRE(id == -1);
}

TEST_CASE("Publish and subscribe roundtrip", "[Session][integration]") {
    Session session;

    if (!session.is_valid()) {
        WARN("Skipping integration test - Zenoh session not available");
        return;
    }

    std::atomic<bool> received{false};
    std::string received_key;
    std::vector<uint8_t> received_payload;

    // Subscribe
    int sub_id = session.subscribe("test/roundtrip", [&](const std::string& key, const std::vector<uint8_t>& payload) {
        received_key = key;
        received_payload = payload;
        received = true;
    });

    REQUIRE(sub_id >= 0);

    // Give subscriber time to establish
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Publish
    std::vector<uint8_t> test_payload = {0xDE, 0xAD, 0xBE, 0xEF};
    REQUIRE(session.publish("test/roundtrip", test_payload));

    // Wait for message
    for (int i = 0; i < 50 && !received; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    REQUIRE(received);
    REQUIRE(received_key == "test/roundtrip");
    REQUIRE(received_payload == test_payload);

    // Cleanup
    session.unsubscribe(sub_id);
}

TEST_CASE("Multiple subscribers on different keys", "[Session][integration]") {
    Session session;

    if (!session.is_valid()) {
        WARN("Skipping integration test - Zenoh session not available");
        return;
    }

    std::atomic<int> count1{0};
    std::atomic<int> count2{0};

    int sub1 = session.subscribe("test/multi/a", [&](const std::string&, const std::vector<uint8_t>&) {
        count1++;
    });

    int sub2 = session.subscribe("test/multi/b", [&](const std::string&, const std::vector<uint8_t>&) {
        count2++;
    });

    REQUIRE(sub1 >= 0);
    REQUIRE(sub2 >= 0);
    REQUIRE(sub1 != sub2);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Publish to both
    session.publish("test/multi/a", {1});
    session.publish("test/multi/b", {2});
    session.publish("test/multi/a", {3});

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    REQUIRE(count1 == 2);
    REQUIRE(count2 == 1);

    session.unsubscribe(sub1);
    session.unsubscribe(sub2);
}

TEST_CASE("Unsubscribe stops receiving messages", "[Session][integration]") {
    Session session;

    if (!session.is_valid()) {
        WARN("Skipping integration test - Zenoh session not available");
        return;
    }

    std::atomic<int> count{0};

    int sub_id = session.subscribe("test/unsub", [&](const std::string&, const std::vector<uint8_t>&) {
        count++;
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // First message should be received
    session.publish("test/unsub", {1});
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(count == 1);

    // Unsubscribe
    session.unsubscribe(sub_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Second message should NOT be received
    session.publish("test/unsub", {2});
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(count == 1);  // Still 1
}

TEST_CASE("Empty payload handling", "[Session][integration]") {
    Session session;

    if (!session.is_valid()) {
        WARN("Skipping integration test - Zenoh session not available");
        return;
    }

    std::atomic<bool> received{false};
    std::vector<uint8_t> received_payload;

    int sub_id = session.subscribe("test/empty", [&](const std::string&, const std::vector<uint8_t>& payload) {
        received_payload = payload;
        received = true;
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Publish empty payload
    std::vector<uint8_t> empty_payload;
    REQUIRE(session.publish("test/empty", empty_payload));

    for (int i = 0; i < 50 && !received; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    REQUIRE(received);
    REQUIRE(received_payload.empty());

    session.unsubscribe(sub_id);
}

TEST_CASE("Large payload handling", "[Session][integration]") {
    Session session;

    if (!session.is_valid()) {
        WARN("Skipping integration test - Zenoh session not available");
        return;
    }

    // Create 1MB payload
    std::vector<uint8_t> large_payload(1024 * 1024);
    for (size_t i = 0; i < large_payload.size(); ++i) {
        large_payload[i] = static_cast<uint8_t>(i % 256);
    }

    std::atomic<bool> received{false};
    size_t received_size = 0;

    int sub_id = session.subscribe("test/large", [&](const std::string&, const std::vector<uint8_t>& payload) {
        received_size = payload.size();
        received = true;
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    REQUIRE(session.publish("test/large", large_payload));

    // Longer wait for large payload
    for (int i = 0; i < 100 && !received; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    REQUIRE(received);
    REQUIRE(received_size == large_payload.size());

    session.unsubscribe(sub_id);
}
