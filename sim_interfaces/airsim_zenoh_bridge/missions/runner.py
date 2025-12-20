"""
Mission Runner

Orchestrates mission execution by running phases in sequence.
"""

import asyncio
import math
import time
from typing import Optional, TYPE_CHECKING

import zenoh

from .config import MissionConfig
from .state import MissionState
from .services.drone import DroneService
from .services.detection import DetectionService
from .phases import get_phase_class, PhaseStatus

if TYPE_CHECKING:
    from .phases.base import MissionPhase


class MissionRunner:
    """
    Mission orchestrator - runs phases in sequence.

    Manages:
    - Zenoh connection
    - Service lifecycle
    - Phase execution
    - Error handling and recovery

    Usage:
        config = MissionConfig.from_yaml("orange_ball.yaml")
        runner = MissionRunner(config, "tcp/localhost:7447")

        if await runner.connect():
            await runner.start_services()
            success = await runner.run()
            await runner.stop_services()
            await runner.disconnect()
    """

    def __init__(self, config: MissionConfig, connect_endpoint: str):
        """
        Initialize mission runner.

        Args:
            config: Mission configuration
            connect_endpoint: Zenoh endpoint (e.g., "tcp/localhost:7447")
        """
        self.config = config
        self.connect_endpoint = connect_endpoint

        self.session: Optional[zenoh.Session] = None
        self.state: Optional[MissionState] = None
        self.drone: Optional[DroneService] = None
        self.detection: Optional[DetectionService] = None

        self._current_phase: Optional['MissionPhase'] = None
        self._start_time = 0.0

    async def connect(self) -> bool:
        """
        Connect to Zenoh.

        Returns:
            True if connected successfully
        """
        try:
            print(f"Connecting to Zenoh at {self.connect_endpoint}...")
            zenoh_config = zenoh.Config()
            zenoh_config.insert_json5("connect/endpoints", f'["{self.connect_endpoint}"]')
            self.session = zenoh.open(zenoh_config)
            print("Connected to Zenoh successfully")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    async def start_services(self) -> None:
        """Initialize and start services."""
        print("Starting services...")

        # Create state
        self.state = MissionState()

        # Copy config params to state for phase access
        self.state.params = {
            'target_class': self.config.target_class,
            'observation_altitude': -self.config.observation_altitude,  # Convert to NED
            'approach_altitude': -self.config.approach_altitude,        # Convert to NED
            'ascent_speed': self.config.ascent_speed,
            'descent_speed': self.config.descent_speed,
            'nav_speed': self.config.nav_speed,
            'approach_speed': self.config.approach_speed,
            'scan_yaw_rate': self.config.scan_yaw_rate,
            'descent_threshold': self.config.descent_threshold,
            'arrival_distance': self.config.arrival_distance,
            'camera_handoff_distance': self.config.camera_handoff_distance,
            'phase_timeout': self.config.phase_timeout,
        }

        # Create services
        self.drone = DroneService(self.session, self.state, self.config.robot_id)
        self.detection = DetectionService(
            self.session, self.state, self.config.robot_id, self.config.cameras
        )

        # Start services
        await self.drone.start()
        await self.detection.start()

        print("Services started")

    async def stop_services(self) -> None:
        """Stop services."""
        print("Stopping services...")
        if self.drone:
            await self.drone.stop()
        if self.detection:
            await self.detection.stop()

    def create_phase(self, phase_config) -> Optional['MissionPhase']:
        """
        Create a phase instance from config.

        Args:
            phase_config: PhaseConfig with type and params

        Returns:
            MissionPhase instance or None if type unknown
        """
        try:
            phase_cls = get_phase_class(phase_config.type)
        except KeyError as e:
            print(f"WARNING: {e}")
            return None

        return phase_cls(
            name=phase_config.name,
            state=self.state,
            drone=self.drone,
            detection=self.detection,
            config=phase_config.params
        )

    async def run(self) -> bool:
        """
        Run the mission.

        Returns:
            True if mission completed successfully
        """
        self._start_time = time.time()

        # Print mission header
        print("\n" + "=" * 60)
        print(f"Mission: {self.config.name}")
        print("=" * 60)
        self.config.print_summary()
        print("=" * 60)

        # Wait for sensor data
        print("\nWaiting for sensor data...")
        odom_ok = await self.drone.wait_for_odom(timeout=10.0)
        det_ok = await self.detection.wait_for_detections(timeout=5.0)

        if not odom_ok:
            print("ERROR: No odometry received!")
            print("  Is the AirSim bridge running?")
            return False

        if not det_ok:
            print("WARNING: No detections received (is detector running?)")
            print("  Continuing anyway...")

        # Print starting pose
        x, y, z, yaw = self.state.pose
        print(f"\nStarting pose: ({x:.1f}, {y:.1f}, {-z:.1f}m) yaw={math.degrees(yaw):.0f}deg")

        success = True

        # Execute phases
        for i, phase_config in enumerate(self.config.phases):
            # Check if disabled
            if not phase_config.enabled:
                print(f"\n[SKIPPED] {phase_config.name}")
                continue

            # Check if mission stopped
            if not self.state.running:
                print("\nMission stopped")
                success = False
                break

            # Check mission timeout
            elapsed = time.time() - self._start_time
            if elapsed > self.config.mission_timeout:
                print(f"\nMission timeout ({self.config.mission_timeout}s)")
                success = False
                break

            # Create and run phase
            phase = self.create_phase(phase_config)
            if not phase:
                continue

            self._current_phase = phase
            result = await phase.run()
            self._current_phase = None

            if result.status == PhaseStatus.FAILED:
                print(f"\nPhase '{phase_config.name}' failed: {result.message}")
                success = False
                break

        # Final status
        self.drone.hover()
        elapsed = time.time() - self._start_time
        x, y, z, yaw = self.state.pose

        print(f"\n{'=' * 60}")
        status = "SUCCEEDED" if success else "INCOMPLETE"
        print(f"Mission {status}")
        print(f"  Final position: ({x:.1f}, {y:.1f}, {-z:.1f}m)")
        print(f"  Elapsed time: {elapsed:.1f}s")

        if self.state.selected_target:
            target = self.state.selected_target
            print(f"  Target: '{target.class_name}' at ({target.world_x:.1f}, {target.world_y:.1f})")

        print("=" * 60)

        return success

    def stop(self) -> None:
        """Stop the mission."""
        print("\nStopping mission...")
        self.state.stop()
        if self._current_phase:
            self._current_phase.cancel()
        if self.drone:
            self.drone.hover()

    async def disconnect(self) -> None:
        """Close Zenoh session."""
        if self.session:
            self.session.close()
            self.session = None
            print("Disconnected from Zenoh")


async def run_mission(config: MissionConfig, connect_endpoint: str) -> bool:
    """
    Convenience function to run a mission.

    Args:
        config: Mission configuration
        connect_endpoint: Zenoh endpoint

    Returns:
        True if mission succeeded
    """
    runner = MissionRunner(config, connect_endpoint)

    if not await runner.connect():
        return False

    try:
        await runner.start_services()
        success = await runner.run()
        return success
    except KeyboardInterrupt:
        print("\nInterrupted.")
        runner.stop()
        return False
    finally:
        await runner.stop_services()
        await runner.disconnect()
