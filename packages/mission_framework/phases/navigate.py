"""
Navigate Phase

Fly to target position at cruise altitude.
"""

import asyncio
import math

from . import register_phase
from .base import MissionPhase, PhaseResult, PhaseStatus


@register_phase("navigate")
class NavigatePhase(MissionPhase):
    """
    Phase to navigate to target position.

    Flies toward the selected target while maintaining cruise altitude.
    Completes when within descent_threshold of target XY position.

    Config params:
        speed: Navigation speed in m/s (overrides mission config)
        arrival_distance: Complete when within this XY distance

    State params used:
        nav_speed: Navigation speed
        descent_threshold: Distance at which to complete phase
        observation_altitude: Altitude to maintain (NED)
    """

    async def execute(self) -> PhaseResult:
        # Check for selected target
        target = self.state.selected_target
        if not target:
            return PhaseResult(
                status=PhaseStatus.FAILED,
                message="No target selected"
            )

        # Get parameters
        speed = self.get_param('speed', self.get_param('nav_speed', 5.0))
        arrival_dist = self.get_param('arrival_distance',
                                      self.get_param('descent_threshold', 10.0))
        target_z = self.get_param('observation_altitude', -25.0)
        timeout = self.get_param('phase_timeout', 60.0)

        target_x = target.world_x
        target_y = target.world_y

        print(f"  Target: ({target_x:.1f}, {target_y:.1f})")
        print(f"  Speed: {speed:.1f}m/s")
        print(f"  Arrival distance: {arrival_dist:.1f}m")

        start_time = asyncio.get_event_loop().time()

        while self.state.running and not self.is_cancelled:
            # Check timeout
            if asyncio.get_event_loop().time() - start_time > timeout:
                self.drone.hover()
                return PhaseResult(
                    status=PhaseStatus.FAILED,
                    message=f"Timeout after {timeout:.0f}s"
                )

            x, y, z, yaw = self.drone.pose

            # Calculate distance and heading to target
            dx = target_x - x
            dy = target_y - y
            dist_xy = math.sqrt(dx * dx + dy * dy)

            # Check if arrived
            if dist_xy < arrival_dist:
                self.drone.hover()
                return PhaseResult(
                    status=PhaseStatus.COMPLETED,
                    message=f"Arrived within {dist_xy:.1f}m of target",
                    data={'final_distance': dist_xy}
                )

            # Calculate velocity toward target
            heading = math.atan2(dy, dx)

            # Scale speed based on distance
            actual_speed = min(speed, max(1.0, dist_xy * 0.3))

            vx = actual_speed * math.cos(heading)
            vy = actual_speed * math.sin(heading)

            # Altitude hold
            alt_error = z - target_z
            vz = alt_error * 2.0  # Simple P controller
            vz = max(-2.0, min(2.0, vz))  # Clamp

            # Yaw toward target
            yaw_error = heading - yaw
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi

            yaw_rate = yaw_error * 1.0  # Simple P controller
            yaw_rate = max(-0.5, min(0.5, yaw_rate))

            self.drone.send_velocity(vx, vy, vz, yaw_rate)

            # Status
            current_alt = -z
            print(f"  Distance: {dist_xy:.1f}m | Alt: {current_alt:.1f}m | "
                  f"Speed: {actual_speed:.1f}m/s", end='\r')

            await asyncio.sleep(0.05)

        self.drone.hover()
        return PhaseResult(
            status=PhaseStatus.FAILED,
            message="Cancelled or stopped"
        )
