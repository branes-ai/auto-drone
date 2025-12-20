"""
Descend Phase

Descend to approach altitude.
"""

import asyncio

from . import register_phase
from .base import MissionPhase, PhaseResult, PhaseStatus


@register_phase("descend")
class DescendPhase(MissionPhase):
    """
    Phase to descend to approach altitude.

    Descends while holding XY position above the target.

    Config params:
        altitude: Target altitude in meters (overrides mission config)
        speed: Descent speed in m/s (overrides mission config)
        tolerance: Altitude tolerance in meters (default: 0.5)

    State params used:
        approach_altitude: Target altitude (NED, negative)
        descent_speed: Descent speed in m/s
    """

    async def execute(self) -> PhaseResult:
        # Get target position
        target = self.state.selected_target
        if not target:
            return PhaseResult(
                status=PhaseStatus.FAILED,
                message="No target selected"
            )

        # Get parameters
        target_z = self.get_param('altitude', None)
        if target_z is not None:
            target_z = -target_z  # Convert to NED
        else:
            target_z = self.get_param('approach_altitude', -5.0)

        speed = self.get_param('speed', self.get_param('descent_speed', 2.0))
        tolerance = self.config.get('tolerance', 0.5)
        timeout = self.get_param('phase_timeout', 60.0)

        target_alt = -target_z
        print(f"  Target altitude: {target_alt:.0f}m")
        print(f"  Descent speed: {speed:.1f}m/s")

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
            current_alt = -z

            # Check if reached target
            if z >= target_z - tolerance:
                self.drone.hover()
                return PhaseResult(
                    status=PhaseStatus.COMPLETED,
                    message=f"Reached {current_alt:.1f}m",
                    data={'final_altitude': current_alt}
                )

            # Hold XY position above target
            import math
            dx = target.world_x - x
            dy = target.world_y - y

            # Position hold with P controller
            vx = dx * 0.5
            vy = dy * 0.5
            vx = max(-2.0, min(2.0, vx))
            vy = max(-2.0, min(2.0, vy))

            # Descend (positive vz = down in NED)
            self.drone.send_velocity(vx, vy, speed, 0)
            print(f"  Altitude: {current_alt:.1f}m / {target_alt:.0f}m", end='\r')

            await asyncio.sleep(0.05)

        self.drone.hover()
        return PhaseResult(
            status=PhaseStatus.FAILED,
            message="Cancelled or stopped"
        )
