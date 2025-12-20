"""
Ascend Phase

Rise to observation altitude.
"""

import asyncio

from . import register_phase
from .base import MissionPhase, PhaseResult, PhaseStatus


@register_phase("ascend")
class AscendPhase(MissionPhase):
    """
    Phase to ascend to observation altitude.

    Config params:
        altitude: Target altitude in meters (overrides mission config)
        speed: Ascent speed in m/s (overrides mission config)
        tolerance: Altitude tolerance in meters (default: 0.5)

    State params used:
        observation_altitude: Target altitude (NED, negative)
        ascent_speed: Ascent speed in m/s
    """

    async def execute(self) -> PhaseResult:
        # Get parameters
        target_z = self.get_param('altitude', None)
        if target_z is not None:
            target_z = -target_z  # Convert to NED
        else:
            target_z = self.get_param('observation_altitude', -25.0)

        speed = self.get_param('speed', self.get_param('ascent_speed', 3.0))
        tolerance = self.config.get('tolerance', 0.5)
        timeout = self.get_param('phase_timeout', 60.0)

        target_alt = -target_z  # For display
        print(f"  Target altitude: {target_alt:.0f}m")
        print(f"  Ascent speed: {speed:.1f}m/s")

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
            if z <= target_z + tolerance:
                self.drone.hover()
                return PhaseResult(
                    status=PhaseStatus.COMPLETED,
                    message=f"Reached {current_alt:.1f}m",
                    data={'final_altitude': current_alt}
                )

            # Ascend (negative vz = up in NED)
            self.drone.send_velocity(0, 0, -speed, 0)
            print(f"  Altitude: {current_alt:.1f}m / {target_alt:.0f}m", end='\r')

            await asyncio.sleep(0.05)

        self.drone.hover()
        return PhaseResult(
            status=PhaseStatus.FAILED,
            message="Cancelled or stopped"
        )
