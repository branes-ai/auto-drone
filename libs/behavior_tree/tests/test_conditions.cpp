#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <behavior_tree/bt_factory.hpp>
#include <behavior_tree/bt_conditions.hpp>
#include <behaviortree_cpp/bt_factory.h>

using namespace autodrone::bt;

TEST_CASE("HasTarget condition", "[bt][conditions]") {
    BT::BehaviorTreeFactory factory;
    registerConditionNodes(factory);

    auto blackboard = BT::Blackboard::create();

    std::string xml = R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="test">
                <HasTarget/>
            </BehaviorTree>
        </root>
    )";

    SECTION("Returns FAILURE when target not selected") {
        blackboard->set("target/selected", false);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::FAILURE);
    }

    SECTION("Returns SUCCESS when target is selected") {
        blackboard->set("target/selected", true);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::SUCCESS);
    }
}

TEST_CASE("HasDetections condition", "[bt][conditions]") {
    BT::BehaviorTreeFactory factory;
    registerConditionNodes(factory);

    auto blackboard = BT::Blackboard::create();

    SECTION("Returns SUCCESS when count >= min_count") {
        std::string xml = R"(
            <root BTCPP_format="4">
                <BehaviorTree ID="test">
                    <HasDetections min_count="1"/>
                </BehaviorTree>
            </root>
        )";

        blackboard->set("inventory/target_count", 3);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::SUCCESS);
    }

    SECTION("Returns FAILURE when count < min_count") {
        std::string xml = R"(
            <root BTCPP_format="4">
                <BehaviorTree ID="test">
                    <HasDetections min_count="5"/>
                </BehaviorTree>
            </root>
        )";

        blackboard->set("inventory/target_count", 3);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::FAILURE);
    }

    SECTION("Returns FAILURE when key not set") {
        std::string xml = R"(
            <root BTCPP_format="4">
                <BehaviorTree ID="test">
                    <HasDetections min_count="1"/>
                </BehaviorTree>
            </root>
        )";

        // Don't set the key
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::FAILURE);
    }
}

TEST_CASE("CheckAltitude condition", "[bt][conditions]") {
    BT::BehaviorTreeFactory factory;
    registerConditionNodes(factory);

    auto blackboard = BT::Blackboard::create();

    std::string xml = R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="test">
                <CheckAltitude target_altitude="25.0" tolerance="0.5"/>
            </BehaviorTree>
        </root>
    )";

    SECTION("Returns SUCCESS when at target altitude") {
        blackboard->set("drone/altitude", 25.0);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::SUCCESS);
    }

    SECTION("Returns SUCCESS when within tolerance") {
        blackboard->set("drone/altitude", 25.3);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::SUCCESS);
    }

    SECTION("Returns FAILURE when below tolerance") {
        blackboard->set("drone/altitude", 20.0);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::FAILURE);
    }

    SECTION("Returns FAILURE when above tolerance") {
        blackboard->set("drone/altitude", 30.0);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::FAILURE);
    }
}

TEST_CASE("CheckDistance condition", "[bt][conditions]") {
    BT::BehaviorTreeFactory factory;
    registerConditionNodes(factory);

    auto blackboard = BT::Blackboard::create();

    std::string xml = R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="test">
                <CheckDistance threshold="10.0"/>
            </BehaviorTree>
        </root>
    )";

    SECTION("Returns SUCCESS when distance <= threshold") {
        blackboard->set("target/distance", 5.0);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::SUCCESS);
    }

    SECTION("Returns SUCCESS when distance == threshold") {
        blackboard->set("target/distance", 10.0);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::SUCCESS);
    }

    SECTION("Returns FAILURE when distance > threshold") {
        blackboard->set("target/distance", 15.0);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::FAILURE);
    }
}

TEST_CASE("CheckMinAltitude condition", "[bt][conditions]") {
    BT::BehaviorTreeFactory factory;
    registerConditionNodes(factory);

    auto blackboard = BT::Blackboard::create();

    std::string xml = R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="test">
                <CheckMinAltitude min_altitude="5.0"/>
            </BehaviorTree>
        </root>
    )";

    SECTION("Returns SUCCESS when altitude >= min") {
        blackboard->set("drone/altitude", 10.0);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::SUCCESS);
    }

    SECTION("Returns FAILURE when altitude < min") {
        blackboard->set("drone/altitude", 3.0);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::FAILURE);
    }
}

TEST_CASE("IsMissionRunning condition", "[bt][conditions]") {
    BT::BehaviorTreeFactory factory;
    registerConditionNodes(factory);

    auto blackboard = BT::Blackboard::create();

    std::string xml = R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="test">
                <IsMissionRunning/>
            </BehaviorTree>
        </root>
    )";

    SECTION("Returns SUCCESS when mission is running") {
        blackboard->set("mission/running", true);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::SUCCESS);
    }

    SECTION("Returns FAILURE when mission is not running") {
        blackboard->set("mission/running", false);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::FAILURE);
    }
}

TEST_CASE("Sequence with conditions", "[bt][conditions]") {
    BT::BehaviorTreeFactory factory;
    registerConditionNodes(factory);

    auto blackboard = BT::Blackboard::create();

    std::string xml = R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="test">
                <Sequence>
                    <HasTarget/>
                    <CheckDistance threshold="10.0"/>
                </Sequence>
            </BehaviorTree>
        </root>
    )";

    SECTION("Sequence succeeds when all conditions pass") {
        blackboard->set("target/selected", true);
        blackboard->set("target/distance", 5.0);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::SUCCESS);
    }

    SECTION("Sequence fails on first condition failure") {
        blackboard->set("target/selected", false);
        blackboard->set("target/distance", 5.0);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::FAILURE);
    }

    SECTION("Sequence fails on second condition failure") {
        blackboard->set("target/selected", true);
        blackboard->set("target/distance", 15.0);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::FAILURE);
    }
}

TEST_CASE("Fallback with conditions", "[bt][conditions]") {
    BT::BehaviorTreeFactory factory;
    registerConditionNodes(factory);

    auto blackboard = BT::Blackboard::create();

    std::string xml = R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="test">
                <Fallback>
                    <HasTarget/>
                    <HasDetections min_count="1"/>
                </Fallback>
            </BehaviorTree>
        </root>
    )";

    SECTION("Fallback succeeds on first condition") {
        blackboard->set("target/selected", true);
        blackboard->set("inventory/target_count", 0);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::SUCCESS);
    }

    SECTION("Fallback tries second condition on first failure") {
        blackboard->set("target/selected", false);
        blackboard->set("inventory/target_count", 3);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::SUCCESS);
    }

    SECTION("Fallback fails when all conditions fail") {
        blackboard->set("target/selected", false);
        blackboard->set("inventory/target_count", 0);
        auto tree = factory.createTreeFromText(xml, blackboard);
        REQUIRE(tree.tickOnce() == BT::NodeStatus::FAILURE);
    }
}
