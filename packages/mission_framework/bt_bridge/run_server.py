#!/usr/bin/env python3
"""
Run the Phase Server for Behavior Tree Integration.

This script starts the Python-side components needed for C++ behavior tree
integration:
  - PhaseServer: Listens for phase requests and executes them
  - StatePublisher: Publishes MissionState for C++ blackboard sync
  - DroneService: Subscribes to odometry, publishes velocity commands
  - DetectionService: Subscribes to detection topics

Usage:
    python -m mission_framework.bt_bridge.run_server --connect tcp/localhost:7447

    # With custom robot ID and cameras
    python -m mission_framework.bt_bridge.run_server \
        --connect tcp/192.168.1.100:7447 \
        --robot-id drone1 \
        --cameras front back down

    # With mission config
    python -m mission_framework.bt_bridge.run_server \
        --connect tcp/localhost:7447 \
        --config configs/orange_ball.yaml
"""

import argparse
import asyncio
import signal
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))


async def main():
    parser = argparse.ArgumentParser(
        description="Run Phase Server for Behavior Tree Integration"
    )
    parser.add_argument(
        "--connect",
        default="tcp/localhost:7447",
        help="Zenoh endpoint to connect to (default: tcp/localhost:7447)"
    )
    parser.add_argument(
        "--robot-id",
        default="drone",
        help="Robot identifier for topic namespacing (default: drone)"
    )
    parser.add_argument(
        "--cameras",
        nargs="+",
        default=["front", "back", "down"],
        help="Camera IDs to subscribe to (default: front back down)"
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to mission config YAML (optional, for default params)"
    )
    parser.add_argument(
        "--publish-rate",
        type=float,
        default=10.0,
        help="State publishing rate in Hz (default: 10)"
    )
    args = parser.parse_args()

    # Import zenoh
    try:
        import zenoh
    except ImportError:
        print("Error: zenoh-python not installed. Install with: pip install eclipse-zenoh")
        sys.exit(1)

    # Import mission framework components
    from mission_framework.state import MissionState
    from mission_framework.services.drone import DroneService
    from mission_framework.services.detection import DetectionService
    from mission_framework.bt_bridge.phase_server import PhaseServer
    from mission_framework.bt_bridge.state_publisher import StatePublisher

    # Load config if provided
    params = {}
    if args.config:
        from mission_framework.config import MissionConfig
        config = MissionConfig.from_yaml(args.config)
        params = {
            "target_class": config.target_class,
            "observation_altitude": config.observation_altitude,
            "approach_altitude": config.approach_altitude,
            "ascent_speed": config.ascent_speed,
            "descent_speed": config.descent_speed,
            "nav_speed": config.nav_speed,
            "approach_speed": config.approach_speed,
            "scan_yaw_rate": config.scan_yaw_rate,
            "descent_threshold": config.descent_threshold,
            "arrival_distance": config.arrival_distance,
            "camera_handoff_distance": config.camera_handoff_distance,
            "mission_timeout": config.mission_timeout,
            "phase_timeout": config.phase_timeout,
        }
        print(f"Loaded config from {args.config}")

    # Connect to Zenoh
    print(f"Connecting to Zenoh at {args.connect}...")
    zenoh_config = zenoh.Config()
    zenoh_config.insert_json5("connect/endpoints", f'["{args.connect}"]')

    session = zenoh.open(zenoh_config)
    print(f"Connected to Zenoh")

    # Create state and services
    state = MissionState(params=params)
    state.running = True  # Mark as running for BT

    drone = DroneService(session, state, args.robot_id)
    detection = DetectionService(session, state, args.robot_id, args.cameras)

    # Start services
    await drone.start()
    await detection.start()

    print("Waiting for odometry...")
    try:
        await drone.wait_for_odom(timeout=30.0)
        print(f"Received odometry: {drone.pose}")
    except TimeoutError:
        print("Warning: No odometry received, continuing anyway")

    # Create and start phase server and state publisher
    phase_server = PhaseServer(session, state, drone, detection, args.robot_id)
    state_publisher = StatePublisher(session, state, args.robot_id, args.publish_rate)

    await phase_server.start()
    state_publisher.start()

    print("\n" + "=" * 60)
    print("Phase Server running. Ready for behavior tree requests.")
    print("Press Ctrl+C to stop.")
    print("=" * 60 + "\n")

    # Wait for interrupt
    stop_event = asyncio.Event()

    def on_signal():
        print("\nShutdown requested...")
        stop_event.set()

    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, on_signal)
        except NotImplementedError:
            # Windows doesn't support add_signal_handler
            pass

    try:
        await stop_event.wait()
    except KeyboardInterrupt:
        pass

    # Cleanup
    print("\nShutting down...")
    state.running = False

    await phase_server.stop()
    state_publisher.stop()
    await drone.stop()
    await detection.stop()

    session.close()
    print("Shutdown complete.")


if __name__ == "__main__":
    asyncio.run(main())
