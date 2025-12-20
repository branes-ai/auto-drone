"""
Drone Control Service

Provides drone control (velocity commands) and state access (odometry).
"""

import asyncio
import time
from typing import Optional, TYPE_CHECKING

import zenoh

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))
from data_types import Odometry, VelocityCommand, CommandPriority, CommandSource

if TYPE_CHECKING:
    from ..state import MissionState


class DroneService:
    """
    Service for drone control and state access.

    Handles:
    - Subscribing to odometry topic
    - Publishing velocity commands
    - Providing thread-safe pose access

    The service updates MissionState.odom when odometry is received.
    """

    def __init__(
        self,
        session: zenoh.Session,
        state: 'MissionState',
        robot_id: str = "drone"
    ):
        """
        Initialize drone service.

        Args:
            session: Zenoh session
            state: Mission state to update
            robot_id: Robot identifier for topics
        """
        self.session = session
        self.state = state
        self.robot_id = robot_id

        # Topics
        self.topic_odom = f"robot/{robot_id}/sensor/state/odom"
        self.topic_cmd_vel = f"robot/{robot_id}/cmd/velocity"

        # Subscribers (stored to prevent GC)
        self._odom_sub = None

        # Statistics
        self.odom_count = 0
        self.cmd_count = 0

    async def start(self) -> None:
        """Start the service - subscribe to odometry topic."""
        self._odom_sub = self.session.declare_subscriber(
            self.topic_odom,
            self._on_odom
        )
        print(f"  DroneService: subscribed to {self.topic_odom}")
        print(f"  DroneService: publishing to {self.topic_cmd_vel}")

    async def stop(self) -> None:
        """Stop the service."""
        # Send final hover command
        self.hover()
        # Subscribers are cleaned up when session closes

    def _on_odom(self, sample: zenoh.Sample) -> None:
        """Handle odometry updates."""
        try:
            odom = Odometry.deserialize(bytes(sample.payload))
            self.state.odom = odom
            self.odom_count += 1
        except Exception:
            pass

    def send_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float,
        priority: CommandPriority = CommandPriority.NORMAL,
        source: CommandSource = CommandSource.MANUAL_CONTROL
    ) -> None:
        """
        Send velocity command (world frame NED).

        Args:
            vx: North velocity (m/s, positive = north)
            vy: East velocity (m/s, positive = east)
            vz: Down velocity (m/s, positive = descend, negative = ascend)
            yaw_rate: Yaw rate (rad/s, positive = clockwise from above)
            priority: Command priority level
            source: Command source identifier
        """
        cmd = VelocityCommand(
            vx=vx,
            vy=vy,
            vz=vz,
            yaw_rate=yaw_rate,
            priority=priority,
            source=source,
            timestamp_us=int(time.time() * 1_000_000)
        )
        self.session.put(self.topic_cmd_vel, cmd.serialize())
        self.cmd_count += 1

    def hover(self) -> None:
        """Send hover command (zero velocity)."""
        self.send_velocity(0, 0, 0, 0)

    def emergency_stop(self) -> None:
        """Send emergency stop command (highest priority)."""
        self.send_velocity(
            0, 0, 0, 0,
            priority=CommandPriority.CRITICAL,
            source=CommandSource.EMERGENCY
        )

    @property
    def pose(self) -> tuple:
        """Get current pose (x, y, z, yaw)."""
        return self.state.pose

    @property
    def position(self) -> tuple:
        """Get current position (x, y, z)."""
        x, y, z, _ = self.state.pose
        return (x, y, z)

    @property
    def altitude(self) -> float:
        """Get current altitude in meters (positive up)."""
        return self.state.altitude

    @property
    def yaw(self) -> float:
        """Get current yaw in radians."""
        _, _, _, yaw = self.state.pose
        return yaw

    async def wait_for_odom(self, timeout: float = 10.0) -> bool:
        """
        Wait for odometry data to be received.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if odometry received, False if timeout
        """
        start = time.time()
        while time.time() - start < timeout:
            if self.state.odom is not None:
                return True
            await asyncio.sleep(0.1)
        return False

    def has_odom(self) -> bool:
        """Check if odometry is available."""
        return self.state.odom is not None
