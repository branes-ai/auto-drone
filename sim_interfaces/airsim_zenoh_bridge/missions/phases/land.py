"""
Land Phase

Controlled landing.
"""

import asyncio

from . import register_phase
from .base import MissionPhase, PhaseResult, PhaseStatus


@register_phase("land")
class LandPhase(MissionPhase):
    """
    Phase for controlled landing.

    Descends at landing speed until reaching minimum altitude.

    Config params:
        speed: Landing speed in m/s (default: 1.0)
        min_altitude: Altitude to stop at (default: 0.5m)
    """

    async def execute(self) -> PhaseResult:
        # Get parameters
        speed = self.config.get('speed', 1.0)
        min_alt = self.config.get('min_altitude', 0.5)
        timeout = self.get_param('phase_timeout', 60.0)

        print(f"  Landing speed: {speed:.1f}m/s")
        print(f"  Min altitude: {min_alt:.1f}m")

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

            # Check if landed
            if current_alt <= min_alt:
                self.drone.hover()
                return PhaseResult(
                    status=PhaseStatus.COMPLETED,
                    message=f"Landed at {current_alt:.1f}m",
                    data={'final_altitude': current_alt}
                )

            # Descend while holding position
            self.drone.send_velocity(0, 0, speed, 0)
            print(f"  Altitude: {current_alt:.1f}m", end='\r')

            await asyncio.sleep(0.05)

        self.drone.hover()
        return PhaseResult(
            status=PhaseStatus.FAILED,
            message="Cancelled or stopped"
        )


@register_phase("hover")
class HoverPhase(MissionPhase):
    """
    Phase to hover in place.

    Config params:
        duration: How long to hover in seconds (default: 5.0)
    """

    async def execute(self) -> PhaseResult:
        duration = self.config.get('duration', 5.0)

        print(f"  Hovering for {duration:.1f}s...")

        start_time = asyncio.get_event_loop().time()

        while self.state.running and not self.is_cancelled:
            elapsed = asyncio.get_event_loop().time() - start_time

            if elapsed >= duration:
                return PhaseResult(
                    status=PhaseStatus.COMPLETED,
                    message=f"Hovered for {elapsed:.1f}s"
                )

            self.drone.hover()
            print(f"  Time: {elapsed:.1f}s / {duration:.1f}s", end='\r')

            await asyncio.sleep(0.1)

        return PhaseResult(
            status=PhaseStatus.FAILED,
            message="Cancelled or stopped"
        )
