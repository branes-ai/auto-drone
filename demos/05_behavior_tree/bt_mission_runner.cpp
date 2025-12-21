/**
 * @file bt_mission_runner.cpp
 * @brief Behavior Tree Mission Runner
 *
 * This demo loads and executes behavior trees that orchestrate mission phases
 * via Zenoh RPC to the Python phase server.
 *
 * Usage:
 *   ./bt_mission_runner --tree trees/orange_ball_mission.xml --connect tcp/localhost:7447
 *
 * Options:
 *   --tree PATH        Path to behavior tree XML file (required)
 *   --connect ENDPOINT Zenoh endpoint to connect to (default: tcp/localhost:7447)
 *   --robot-id ID      Robot identifier (default: drone)
 *   --tick-rate HZ     Tree tick rate in Hz (default: 20)
 *   --help             Show this help message
 *
 * Before running:
 *   1. Start Zenoh router (or use peer mode)
 *   2. Start Python phase server:
 *      python -m mission_framework.bt_bridge.run_server --connect tcp/localhost:7447
 *   3. Start simulator (AirSim) or mock publisher
 */

#include <behavior_tree/bt_factory.hpp>
#include <behavior_tree/bt_blackboard_sync.hpp>
#include <ZenohInterface.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include <iostream>
#include <filesystem>
#include <csignal>
#include <thread>
#include <chrono>

namespace fs = std::filesystem;

// Global flag for signal handling
std::atomic<bool> g_running{true};

void signalHandler(int /*signum*/) {
    std::cout << "\nInterrupt received, stopping...\n";
    g_running = false;
}

void printUsage(const char* program) {
    std::cout << "Usage: " << program << " [options]\n\n"
              << "Options:\n"
              << "  --tree PATH        Path to behavior tree XML file (required)\n"
              << "  --connect ENDPOINT Zenoh endpoint (default: tcp/localhost:7447)\n"
              << "  --robot-id ID      Robot identifier (default: drone)\n"
              << "  --tick-rate HZ     Tree tick rate in Hz (default: 20)\n"
              << "  --help             Show this help message\n"
              << "\nExample:\n"
              << "  " << program << " --tree trees/orange_ball_mission.xml\n";
}

int main(int argc, char* argv[]) {
    // Default arguments
    std::string tree_file;
    std::string zenoh_endpoint = "tcp/localhost:7447";
    std::string robot_id = "drone";
    double tick_rate_hz = 20.0;

    // Parse arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--tree" && i + 1 < argc) {
            tree_file = argv[++i];
        } else if (arg == "--connect" && i + 1 < argc) {
            zenoh_endpoint = argv[++i];
        } else if (arg == "--robot-id" && i + 1 < argc) {
            robot_id = argv[++i];
        } else if (arg == "--tick-rate" && i + 1 < argc) {
            tick_rate_hz = std::stod(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    // Validate required arguments
    if (tree_file.empty()) {
        std::cerr << "Error: --tree argument is required\n\n";
        printUsage(argv[0]);
        return 1;
    }

    // Check if tree file exists
    if (!fs::exists(tree_file)) {
        std::cerr << "Error: Tree file not found: " << tree_file << "\n";
        return 1;
    }

    // Install signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "=== Behavior Tree Mission Runner ===\n\n";
    std::cout << "Tree file: " << tree_file << "\n";
    std::cout << "Zenoh endpoint: " << zenoh_endpoint << "\n";
    std::cout << "Robot ID: " << robot_id << "\n";
    std::cout << "Tick rate: " << tick_rate_hz << " Hz\n";
    std::cout << "\n";

    // Connect to Zenoh
    std::cout << "Connecting to Zenoh...\n";
    auto session = std::make_shared<zenoh_interface::Session>(
        zenoh_interface::SessionConfig::connect_to(zenoh_endpoint)
    );

    if (!session->is_valid()) {
        std::cerr << "Error: Failed to connect to Zenoh at " << zenoh_endpoint << "\n";
        return 1;
    }
    std::cout << "Connected to Zenoh\n";

    // Create BT factory and register nodes
    BT::BehaviorTreeFactory factory;
    autodrone::bt::FactoryConfig config(session, robot_id);
    autodrone::bt::registerAllNodes(factory, config);

    std::cout << "Registered custom BT nodes\n";

    // Create blackboard
    auto blackboard = BT::Blackboard::create();

    // Setup blackboard sync with Zenoh
    autodrone::bt::BlackboardSync bb_sync(blackboard, session, robot_id);
    bb_sync.start();

    std::cout << "Started blackboard sync\n";

    // Wait for odometry
    std::cout << "Waiting for odometry...\n";
    int wait_count = 0;
    while (!bb_sync.hasOdom() && g_running && wait_count < 100) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_count++;
    }

    if (!bb_sync.hasOdom()) {
        std::cout << "Warning: No odometry received, continuing anyway\n";
    } else {
        std::cout << "Received odometry\n";
    }

    // Load behavior tree
    std::cout << "Loading tree from: " << tree_file << "\n";
    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(tree_file, blackboard);
    } catch (const std::exception& e) {
        std::cerr << "Error loading tree: " << e.what() << "\n";
        bb_sync.stop();
        return 1;
    }
    std::cout << "Tree loaded successfully\n";

    // Calculate tick interval
    auto tick_interval = std::chrono::microseconds(
        static_cast<int64_t>(1000000.0 / tick_rate_hz)
    );

    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << " Starting behavior tree execution\n";
    std::cout << " Press Ctrl+C to stop\n";
    std::cout << "========================================\n\n";

    // Main execution loop
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    auto start_time = std::chrono::steady_clock::now();

    while (g_running && status == BT::NodeStatus::RUNNING) {
        auto tick_start = std::chrono::steady_clock::now();

        // Update computed blackboard values
        bb_sync.updateComputed();

        // Tick the tree
        status = tree.tickOnce();

        // Sleep to maintain tick rate
        auto tick_end = std::chrono::steady_clock::now();
        auto elapsed = tick_end - tick_start;
        if (elapsed < tick_interval) {
            std::this_thread::sleep_for(tick_interval - elapsed);
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    auto total_elapsed = std::chrono::duration<double>(end_time - start_time).count();

    // Report result
    std::cout << "\n========================================\n";
    std::cout << " Behavior tree execution finished\n";
    std::cout << "========================================\n\n";

    std::cout << "Status: ";
    switch (status) {
        case BT::NodeStatus::SUCCESS:
            std::cout << "SUCCESS\n";
            break;
        case BT::NodeStatus::FAILURE:
            std::cout << "FAILURE\n";
            break;
        case BT::NodeStatus::RUNNING:
            std::cout << "INTERRUPTED (was still running)\n";
            break;
        default:
            std::cout << "UNKNOWN\n";
    }

    std::cout << "Total time: " << total_elapsed << " seconds\n";

    // Cleanup
    std::cout << "\nCleaning up...\n";
    bb_sync.stop();

    std::cout << "Done.\n";
    return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
