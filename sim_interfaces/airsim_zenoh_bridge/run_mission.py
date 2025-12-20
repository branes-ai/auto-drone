#!/usr/bin/env python3
"""
Mission Runner CLI

Run drone missions defined in YAML configuration files.

Usage:
    python run_mission.py --connect tcp/localhost:7447 --config orange_ball.yaml
    python run_mission.py --connect tcp/localhost:7447 --config orange_ball.yaml --altitude 30
    python run_mission.py --connect tcp/localhost:7447 --target "red cube"

Examples:
    # Run with config file
    python run_mission.py --connect tcp/192.168.1.10:7447 \\
        --config orange_ball.yaml

    # Override altitude
    python run_mission.py --connect tcp/localhost:7447 \\
        --config orange_ball.yaml --altitude 30

    # Quick test with default mission
    python run_mission.py --connect tcp/localhost:7447

Requirements:
    - AirSim bridge running (airsim_zenoh_bridge.py)
    - YOLO detector running (yolo_multi_camera_detector.py)
"""

import argparse
import asyncio
import sys
from pathlib import Path

# Add paths for imports
_this_dir = Path(__file__).parent
_project_root = _this_dir.parent.parent
sys.path.insert(0, str(_this_dir))  # For data_types
sys.path.insert(0, str(_project_root / "packages"))  # For mission_framework

from mission_framework.config import MissionConfig
from mission_framework.runner import MissionRunner

# Import phases to trigger registration
from mission_framework.phases import (
    ascend, scan, select, navigate, descend, approach, land
)
from mission_framework.phases import list_phases

# Default config location
DEFAULT_CONFIG_DIR = _project_root / "packages" / "mission_framework" / "configs"


def main():
    parser = argparse.ArgumentParser(
        description="Run a drone mission from YAML configuration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run orange ball mission with config file
  python run_mission.py --connect tcp/localhost:7447 --config orange_ball.yaml

  # Override observation altitude
  python run_mission.py --connect tcp/localhost:7447 \\
      --config orange_ball.yaml --altitude 30

  # Different target
  python run_mission.py --connect tcp/localhost:7447 \\
      --config orange_ball.yaml --target "red cube"

  # Quick test with default config
  python run_mission.py --connect tcp/localhost:7447

  # List available phases
  python run_mission.py --list-phases

Available phases: """ + ", ".join(list_phases())
    )

    parser.add_argument("--connect", required=True,
                        help="Zenoh endpoint (e.g., tcp/localhost:7447)")
    parser.add_argument("--config", "-c",
                        help="Mission config YAML file")
    parser.add_argument("--robot-id",
                        help="Override robot ID")
    parser.add_argument("--target",
                        help="Override target class")
    parser.add_argument("--altitude", type=float,
                        help="Override observation altitude (m)")
    parser.add_argument("--approach", type=float,
                        help="Override approach altitude (m)")
    parser.add_argument("--timeout", type=float,
                        help="Override mission timeout (s)")
    parser.add_argument("--list-phases", action="store_true",
                        help="List available phases and exit")

    args = parser.parse_args()

    if args.list_phases:
        print("Available phases:")
        for phase in list_phases():
            print(f"  - {phase}")
        return 0

    # Load config
    if args.config:
        config_path = Path(args.config)
        if not config_path.exists():
            # Try default config directory
            alt_path = DEFAULT_CONFIG_DIR / args.config
            if alt_path.exists():
                config_path = alt_path
            else:
                print(f"ERROR: Config file not found: {args.config}")
                print(f"  Searched: {args.config}")
                print(f"  Searched: {alt_path}")
                return 1

        config = MissionConfig.from_yaml(config_path)
        print(f"Loaded config: {config_path}")
    else:
        # Use default orange ball config
        config = MissionConfig.default_orange_ball()
        print("Using default orange ball mission config")

    # Apply CLI overrides
    if args.robot_id:
        config.robot_id = args.robot_id
    if args.target:
        config.target_class = args.target
    if args.altitude:
        config.observation_altitude = args.altitude
    if args.approach:
        config.approach_altitude = args.approach
    if args.timeout:
        config.mission_timeout = args.timeout

    # Run mission
    async def run():
        runner = MissionRunner(config, args.connect)

        if not await runner.connect():
            return 1

        try:
            await runner.start_services()
            success = await runner.run()
            return 0 if success else 1
        except KeyboardInterrupt:
            print("\nInterrupted.")
            runner.stop()
            return 1
        finally:
            await runner.stop_services()
            await runner.disconnect()

    return asyncio.run(run())


if __name__ == "__main__":
    sys.exit(main())
