#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "PIDController.hpp"
#include <cmath>

using namespace control_algorithms;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

TEST_CASE("PIDController basic functionality", "[pid]") {
    SECTION("Default construction") {
        PIDController pid;
        REQUIRE(pid.config().gains.kp == 1.0f);
        REQUIRE(pid.config().gains.ki == 0.0f);
        REQUIRE(pid.config().gains.kd == 0.0f);
    }

    SECTION("Construction with gains") {
        PIDController pid(2.0f, 0.1f, 0.5f);
        REQUIRE(pid.config().gains.kp == 2.0f);
        REQUIRE(pid.config().gains.ki == 0.1f);
        REQUIRE(pid.config().gains.kd == 0.5f);
    }

    SECTION("Proportional only") {
        PIDController pid(1.0f, 0.0f, 0.0f);

        // Error of 5.0, kp of 1.0 -> output of 5.0
        float output = pid.update(0.01f, 10.0f, 5.0f);
        REQUIRE_THAT(output, WithinAbs(1.0f, 0.01f));  // Clamped to default max of 1.0

        // With higher limits
        PIDController::Limits limits;
        limits.output_min = -10.0f;
        limits.output_max = 10.0f;
        pid.set_limits(limits);
        pid.reset();

        output = pid.update(0.01f, 10.0f, 5.0f);
        REQUIRE_THAT(output, WithinAbs(5.0f, 0.01f));
    }

    SECTION("Integral accumulation") {
        PIDController pid(0.0f, 1.0f, 0.0f);
        PIDController::Limits limits;
        limits.output_min = -100.0f;
        limits.output_max = 100.0f;
        limits.integral_min = -100.0f;
        limits.integral_max = 100.0f;
        pid.set_limits(limits);

        float dt = 0.01f;
        float error = 10.0f;

        // After 10 updates: integral = error * dt * 10 = 10 * 0.01 * 10 = 1.0
        for (int i = 0; i < 10; ++i) {
            pid.update(dt, error, 0.0f);
        }

        REQUIRE_THAT(pid.integral(), WithinAbs(1.0f, 0.01f));
    }

    SECTION("Anti-windup") {
        PIDController pid(0.0f, 10.0f, 0.0f);  // High ki to quickly saturate
        PIDController::Limits limits;
        limits.integral_min = -5.0f;
        limits.integral_max = 5.0f;
        limits.output_min = -100.0f;
        limits.output_max = 100.0f;
        pid.set_limits(limits);

        // Large error for many iterations should hit integral limit
        for (int i = 0; i < 100; ++i) {
            pid.update(0.1f, 100.0f, 0.0f);
        }

        REQUIRE(pid.integral() <= 5.0f);
        REQUIRE(pid.integral() >= -5.0f);
    }

    SECTION("Reset clears state") {
        PIDController pid(1.0f, 1.0f, 1.0f);

        // Run some updates
        for (int i = 0; i < 10; ++i) {
            pid.update(0.01f, 10.0f, 5.0f);
        }

        REQUIRE(pid.integral() != 0.0f);

        pid.reset();

        REQUIRE(pid.integral() == 0.0f);
        REQUIRE(pid.error() == 0.0f);
        REQUIRE(pid.derivative() == 0.0f);
    }

    SECTION("Zero dt returns zero") {
        PIDController pid(1.0f, 1.0f, 1.0f);
        float output = pid.update(0.0f, 10.0f, 5.0f);
        REQUIRE(output == 0.0f);
    }
}

TEST_CASE("PositionController", "[pid][position]") {
    SECTION("Uniform gains") {
        PositionController pos(1.0f, 0.0f, 0.0f);

        // Set higher limits
        PIDController::Limits limits;
        limits.output_min = -10.0f;
        limits.output_max = 10.0f;
        pos.x().set_limits(limits);
        pos.y().set_limits(limits);
        pos.z().set_limits(limits);

        float vx, vy, vz;
        pos.update(0.01f,
                   10.0f, 20.0f, 30.0f,   // target
                   5.0f, 15.0f, 25.0f,    // current
                   vx, vy, vz);

        // Errors are all 5.0, with kp=1.0
        REQUIRE_THAT(vx, WithinAbs(5.0f, 0.01f));
        REQUIRE_THAT(vy, WithinAbs(5.0f, 0.01f));
        REQUIRE_THAT(vz, WithinAbs(5.0f, 0.01f));
    }

    SECTION("Reset clears all axes") {
        PositionController pos(1.0f, 1.0f, 1.0f);

        float vx, vy, vz;
        for (int i = 0; i < 10; ++i) {
            pos.update(0.01f, 10.0f, 20.0f, 30.0f, 5.0f, 15.0f, 25.0f, vx, vy, vz);
        }

        pos.reset();

        REQUIRE(pos.x().integral() == 0.0f);
        REQUIRE(pos.y().integral() == 0.0f);
        REQUIRE(pos.z().integral() == 0.0f);
    }
}

TEST_CASE("YawController", "[pid][yaw]") {
    SECTION("Basic yaw control") {
        YawController yaw(1.0f, 0.0f, 0.0f);

        // Target yaw 0, current yaw 0.5 -> error is -0.5
        float output = yaw.update(0.01f, 0.0f, 0.5f);
        REQUIRE_THAT(output, WithinAbs(-0.5f, 0.01f));
    }

    SECTION("Angle wrapping - shortest path") {
        YawController yaw(1.0f, 0.0f, 0.0f);

        constexpr float PI = 3.14159265358979323846f;

        // Target: -170 deg, Current: 170 deg
        // Error = target - current = -170 - 170 = -340 deg, wrapped to +20 deg
        // So the controller should output a positive value (turn right/CW)
        float target = -170.0f * PI / 180.0f;  // -2.967 rad
        float current = 170.0f * PI / 180.0f;  // 2.967 rad

        float output = yaw.update(0.01f, target, current);

        // The wrapped error is +20 degrees (+0.349 rad)
        float expected_error = 20.0f * PI / 180.0f;
        REQUIRE_THAT(output, WithinAbs(expected_error, 0.02f));
    }

    SECTION("Wrapping at PI boundary") {
        YawController yaw(1.0f, 0.0f, 0.0f);

        constexpr float PI = 3.14159265358979323846f;

        // Current at PI, target at -PI -> should be zero error
        float output = yaw.update(0.01f, -PI, PI);
        REQUIRE_THAT(output, WithinAbs(0.0f, 0.01f));
    }
}

TEST_CASE("PID convergence simulation", "[pid][simulation]") {
    SECTION("Position setpoint tracking") {
        // PID tuned for a damped point mass system
        PIDController pid(2.0f, 0.5f, 1.0f);
        PIDController::Limits limits;
        limits.output_min = -10.0f;
        limits.output_max = 10.0f;
        limits.integral_min = -20.0f;
        limits.integral_max = 20.0f;
        pid.set_limits(limits);

        float position = 0.0f;
        float velocity = 0.0f;
        float target = 10.0f;
        float dt = 0.01f;
        float mass = 1.0f;
        float damping = 0.5f;

        // Simulate for 20 seconds
        for (int i = 0; i < 2000; ++i) {
            float force = pid.update(dt, target, position);
            float accel = (force - damping * velocity) / mass;
            velocity += accel * dt;
            position += velocity * dt;
        }

        // Should converge close to target
        REQUIRE_THAT(position, WithinAbs(target, 0.5f));
    }
}
