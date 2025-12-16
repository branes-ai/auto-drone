#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "ReactiveAvoidance.hpp"
#include "ProximityData.hpp"
#include <cmath>

using namespace control_algorithms;
using namespace data_types;
using Catch::Matchers::WithinAbs;

TEST_CASE("ReactiveAvoidance basic functionality", "[avoidance]") {
    SECTION("Default construction") {
        ReactiveAvoidance avoidance;
        REQUIRE(avoidance.config().safety_distance == 2.0f);
        REQUIRE(avoidance.config().critical_distance == 0.5f);
        REQUIRE(avoidance.config().max_avoidance_speed == 2.0f);
        REQUIRE(avoidance.config().gain == 1.5f);
    }

    SECTION("Construction with config") {
        ReactiveAvoidance::Config config;
        config.safety_distance = 3.0f;
        config.critical_distance = 1.0f;
        config.max_avoidance_speed = 5.0f;
        config.gain = 2.0f;

        ReactiveAvoidance avoidance(config);
        REQUIRE(avoidance.config().safety_distance == 3.0f);
        REQUIRE(avoidance.config().critical_distance == 1.0f);
        REQUIRE(avoidance.config().max_avoidance_speed == 5.0f);
        REQUIRE(avoidance.config().gain == 2.0f);
    }
}

TEST_CASE("ReactiveAvoidance CLEAR state", "[avoidance]") {
    ReactiveAvoidance avoidance;
    float vx, vy, vz;

    SECTION("No obstacles (clear readings)") {
        ProximityData prox = ProximityData::clear();
        auto state = avoidance.update(prox, vx, vy, vz);

        REQUIRE(state == ReactiveAvoidance::State::CLEAR);
        REQUIRE_THAT(vx, WithinAbs(0.0f, 0.001f));
        REQUIRE_THAT(vy, WithinAbs(0.0f, 0.001f));
        REQUIRE_THAT(vz, WithinAbs(0.0f, 0.001f));
    }

    SECTION("All obstacles beyond safety distance") {
        ProximityData prox(5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 0);
        auto state = avoidance.update(prox, vx, vy, vz);

        REQUIRE(state == ReactiveAvoidance::State::CLEAR);
        REQUIRE_THAT(vx, WithinAbs(0.0f, 0.001f));
        REQUIRE_THAT(vy, WithinAbs(0.0f, 0.001f));
        REQUIRE_THAT(vz, WithinAbs(0.0f, 0.001f));
    }

    SECTION("Exactly at safety distance") {
        ProximityData prox(2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 0);
        auto state = avoidance.update(prox, vx, vy, vz);

        REQUIRE(state == ReactiveAvoidance::State::CLEAR);
    }
}

TEST_CASE("ReactiveAvoidance CRITICAL state", "[avoidance]") {
    ReactiveAvoidance avoidance;
    float vx, vy, vz;

    SECTION("Obstacle below critical distance") {
        ProximityData prox(0.3f, 999.0f, 999.0f, 999.0f, 999.0f, 999.0f, 0);
        auto state = avoidance.update(prox, vx, vy, vz);

        REQUIRE(state == ReactiveAvoidance::State::CRITICAL);
        // Velocities should be zero in critical state
        REQUIRE_THAT(vx, WithinAbs(0.0f, 0.001f));
        REQUIRE_THAT(vy, WithinAbs(0.0f, 0.001f));
        REQUIRE_THAT(vz, WithinAbs(0.0f, 0.001f));
    }

    SECTION("Multiple directions at critical") {
        ProximityData prox(0.2f, 0.3f, 999.0f, 999.0f, 999.0f, 999.0f, 0);
        auto state = avoidance.update(prox, vx, vy, vz);

        REQUIRE(state == ReactiveAvoidance::State::CRITICAL);
    }
}

TEST_CASE("ReactiveAvoidance AVOIDING state", "[avoidance]") {
    ReactiveAvoidance avoidance;
    float vx, vy, vz;

    SECTION("Obstacle in front - escape backward") {
        ProximityData prox(1.0f, 999.0f, 999.0f, 999.0f, 999.0f, 999.0f, 0);
        auto state = avoidance.update(prox, vx, vy, vz);

        REQUIRE(state == ReactiveAvoidance::State::AVOIDING);
        REQUIRE(vx < 0.0f);  // Negative vx = backward
        REQUIRE_THAT(vy, WithinAbs(0.0f, 0.001f));
        REQUIRE_THAT(vz, WithinAbs(0.0f, 0.001f));
    }

    SECTION("Obstacle behind - escape forward") {
        ProximityData prox(999.0f, 1.0f, 999.0f, 999.0f, 999.0f, 999.0f, 0);
        auto state = avoidance.update(prox, vx, vy, vz);

        REQUIRE(state == ReactiveAvoidance::State::AVOIDING);
        REQUIRE(vx > 0.0f);  // Positive vx = forward
        REQUIRE_THAT(vy, WithinAbs(0.0f, 0.001f));
        REQUIRE_THAT(vz, WithinAbs(0.0f, 0.001f));
    }

    SECTION("Obstacle on left - escape right") {
        ProximityData prox(999.0f, 999.0f, 1.0f, 999.0f, 999.0f, 999.0f, 0);
        auto state = avoidance.update(prox, vx, vy, vz);

        REQUIRE(state == ReactiveAvoidance::State::AVOIDING);
        REQUIRE_THAT(vx, WithinAbs(0.0f, 0.001f));
        REQUIRE(vy > 0.0f);  // Positive vy = right
        REQUIRE_THAT(vz, WithinAbs(0.0f, 0.001f));
    }

    SECTION("Obstacle on right - escape left") {
        ProximityData prox(999.0f, 999.0f, 999.0f, 1.0f, 999.0f, 999.0f, 0);
        auto state = avoidance.update(prox, vx, vy, vz);

        REQUIRE(state == ReactiveAvoidance::State::AVOIDING);
        REQUIRE_THAT(vx, WithinAbs(0.0f, 0.001f));
        REQUIRE(vy < 0.0f);  // Negative vy = left
        REQUIRE_THAT(vz, WithinAbs(0.0f, 0.001f));
    }

    SECTION("Obstacle above - escape down") {
        ProximityData prox(999.0f, 999.0f, 999.0f, 999.0f, 1.0f, 999.0f, 0);
        auto state = avoidance.update(prox, vx, vy, vz);

        REQUIRE(state == ReactiveAvoidance::State::AVOIDING);
        REQUIRE_THAT(vx, WithinAbs(0.0f, 0.001f));
        REQUIRE_THAT(vy, WithinAbs(0.0f, 0.001f));
        REQUIRE(vz > 0.0f);  // Positive vz = down
    }

    SECTION("Obstacle below - escape up") {
        ProximityData prox(999.0f, 999.0f, 999.0f, 999.0f, 999.0f, 1.0f, 0);
        auto state = avoidance.update(prox, vx, vy, vz);

        REQUIRE(state == ReactiveAvoidance::State::AVOIDING);
        REQUIRE_THAT(vx, WithinAbs(0.0f, 0.001f));
        REQUIRE_THAT(vy, WithinAbs(0.0f, 0.001f));
        REQUIRE(vz < 0.0f);  // Negative vz = up
    }
}

TEST_CASE("ReactiveAvoidance multi-direction avoidance", "[avoidance]") {
    ReactiveAvoidance avoidance;
    float vx, vy, vz;

    SECTION("Front and left obstacles - escape back-right") {
        ProximityData prox(1.0f, 999.0f, 1.0f, 999.0f, 999.0f, 999.0f, 0);
        auto state = avoidance.update(prox, vx, vy, vz);

        REQUIRE(state == ReactiveAvoidance::State::AVOIDING);
        REQUIRE(vx < 0.0f);  // Backward
        REQUIRE(vy > 0.0f);  // Right
    }

    SECTION("Symmetric front-back cancellation") {
        // Equal distance front and back should cancel X velocity
        ProximityData prox(1.5f, 1.5f, 999.0f, 999.0f, 999.0f, 999.0f, 0);
        auto state = avoidance.update(prox, vx, vy, vz);

        REQUIRE(state == ReactiveAvoidance::State::AVOIDING);
        REQUIRE_THAT(vx, WithinAbs(0.0f, 0.001f));
    }
}

TEST_CASE("ReactiveAvoidance velocity clamping", "[avoidance]") {
    ReactiveAvoidance::Config config;
    config.max_avoidance_speed = 1.0f;
    config.gain = 10.0f;  // High gain to ensure clamping
    ReactiveAvoidance avoidance(config);
    float vx, vy, vz;

    SECTION("Single direction clamped") {
        ProximityData prox(0.6f, 999.0f, 999.0f, 999.0f, 999.0f, 999.0f, 0);
        avoidance.update(prox, vx, vy, vz);

        float magnitude = std::sqrt(vx * vx + vy * vy + vz * vz);
        REQUIRE(magnitude <= config.max_avoidance_speed + 0.01f);
    }

    SECTION("Multi-direction clamped") {
        ProximityData prox(0.6f, 999.0f, 0.6f, 999.0f, 999.0f, 999.0f, 0);
        avoidance.update(prox, vx, vy, vz);

        float magnitude = std::sqrt(vx * vx + vy * vy + vz * vz);
        REQUIRE(magnitude <= config.max_avoidance_speed + 0.01f);
    }
}

TEST_CASE("ReactiveAvoidance closer obstacle stronger response", "[avoidance]") {
    ReactiveAvoidance avoidance;
    float vx1, vy1, vz1;
    float vx2, vy2, vz2;

    // Closer obstacle should generate larger escape velocity
    ProximityData far_prox(1.8f, 999.0f, 999.0f, 999.0f, 999.0f, 999.0f, 0);
    ProximityData near_prox(1.0f, 999.0f, 999.0f, 999.0f, 999.0f, 999.0f, 0);

    avoidance.update(far_prox, vx1, vy1, vz1);
    avoidance.update(near_prox, vx2, vy2, vz2);

    // Near obstacle should produce larger escape velocity
    REQUIRE(std::abs(vx2) > std::abs(vx1));
}

TEST_CASE("ReactiveAvoidance reset", "[avoidance]") {
    ReactiveAvoidance avoidance;
    float vx, vy, vz;

    // Generate an avoiding state
    ProximityData prox(1.0f, 999.0f, 999.0f, 999.0f, 999.0f, 999.0f, 0);
    avoidance.update(prox, vx, vy, vz);
    REQUIRE(avoidance.last_state() == ReactiveAvoidance::State::AVOIDING);

    // Reset
    avoidance.reset();
    REQUIRE(avoidance.last_state() == ReactiveAvoidance::State::CLEAR);
    REQUIRE_THAT(avoidance.last_escape_magnitude(), WithinAbs(0.0f, 0.001f));
}
