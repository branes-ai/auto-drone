"""
Approach Phase

Final approach with visual servoing and camera handoff.
"""

import asyncio
import math
import time

from . import register_phase
from .base import MissionPhase, PhaseResult, PhaseStatus


@register_phase("approach")
class ApproachPhase(MissionPhase):
    """
    Phase for final approach with visual servoing.

    Uses live detections to guide approach. Supports camera handoff
    from front to down camera when close to target.

    Config params:
        arrival_distance: Complete when within this distance (default: 3.0m)
        handoff_distance: Switch to down camera when within (default: 8.0m)
        use_down_camera: Whether to use down camera for final approach

    State params used:
        approach_speed: Approach speed
        arrival_distance: Complete distance
        camera_handoff_distance: Handoff distance
        target_class: Object to track
    """

    async def execute(self) -> PhaseResult:
        # Get parameters
        target_class = self.get_param('target_class', 'orange ball')
        arrival_dist = self.get_param('arrival_distance',
                                      self.get_param('arrival_distance', 3.0))
        handoff_dist = self.get_param('handoff_distance',
                                      self.get_param('camera_handoff_distance', 8.0))
        approach_speed = self.get_param('speed',
                                        self.get_param('approach_speed', 2.0))
        use_down = self.config.get('use_down_camera', True)
        timeout = self.get_param('phase_timeout', 60.0)

        # Check for target
        target = self.state.selected_target
        if not target:
            return PhaseResult(
                status=PhaseStatus.FAILED,
                message="No target selected"
            )

        print(f"  Target: '{target_class}'")
        print(f"  Arrival distance: {arrival_dist:.1f}m")
        if use_down and "down" in self.detection.cameras:
            print(f"  Camera handoff at: {handoff_dist:.1f}m")

        # Tracking state
        active_camera = "front"
        lost_count = 0
        max_lost = 20  # ~1 second at 20Hz

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

            # Calculate distance to last known target position
            dx = target.world_x - x
            dy = target.world_y - y
            dist_xy = math.sqrt(dx * dx + dy * dy)

            # Check arrival
            if dist_xy < arrival_dist:
                self.drone.hover()
                return PhaseResult(
                    status=PhaseStatus.COMPLETED,
                    message=f"Arrived within {dist_xy:.1f}m of target",
                    data={'final_distance': dist_xy}
                )

            # Determine which camera to use
            if use_down and "down" in self.detection.cameras and dist_xy < handoff_dist:
                active_camera = "down"
            else:
                active_camera = "front" if "front" in self.detection.cameras else self.detection.cameras[0]

            # Get detection from active camera
            camera_id, detection = self.detection.get_best_detection(target_class)

            if detection and camera_id == active_camera:
                lost_count = 0

                # Visual servoing based on bearing
                bearing_x = detection.bearing_x  # -1 to 1 (left to right)
                bearing_y = detection.bearing_y  # -1 to 1 (top to bottom)

                if active_camera == "down":
                    # Down camera: bearing_x/y map to XY velocity
                    # Bearing_x positive = target to right of center = move right (positive Y in NED)
                    # Bearing_y positive = target below center = move forward (positive X in NED)
                    vx = bearing_y * approach_speed
                    vy = bearing_x * approach_speed

                    # Descend slowly
                    vz = 0.5

                    # Hold yaw
                    yaw_rate = 0
                else:
                    # Front camera: bearing_x controls yaw, bearing_y controls altitude
                    # Move forward at approach speed
                    vx = approach_speed * math.cos(yaw)
                    vy = approach_speed * math.sin(yaw)

                    # Yaw to center target
                    yaw_rate = bearing_x * 1.0
                    yaw_rate = max(-0.5, min(0.5, yaw_rate))

                    # Adjust altitude based on vertical bearing
                    vz = bearing_y * 1.0
                    vz = max(-1.0, min(1.0, vz))

                self.drone.send_velocity(vx, vy, vz, yaw_rate)

                status = f"[{active_camera}] dist={dist_xy:.1f}m bearing=({bearing_x:.2f}, {bearing_y:.2f})"
            else:
                # No detection - use dead reckoning
                lost_count += 1

                if lost_count > max_lost:
                    # Lost target - hover and fail
                    self.drone.hover()
                    return PhaseResult(
                        status=PhaseStatus.FAILED,
                        message=f"Lost target for {lost_count} frames"
                    )

                # Continue toward last known position
                heading = math.atan2(dy, dx)
                vx = approach_speed * 0.5 * math.cos(heading)
                vy = approach_speed * 0.5 * math.sin(heading)

                self.drone.send_velocity(vx, vy, 0, 0)
                status = f"[LOST x{lost_count}] dist={dist_xy:.1f}m"

            print(f"  {status}", end='\r')
            await asyncio.sleep(0.05)

        self.drone.hover()
        return PhaseResult(
            status=PhaseStatus.FAILED,
            message="Cancelled or stopped"
        )
