"""
Scan Phase

Perform rotational scan to detect objects.
"""

import asyncio
import math
import time

from . import register_phase
from .base import MissionPhase, PhaseResult, PhaseStatus
from ..state import TrackedObject


@register_phase("scan")
class ScanPhase(MissionPhase):
    """
    Phase to perform rotational scan for object detection.

    Rotates the drone while collecting detections from all cameras.
    With front+back cameras, 180° rotation provides 360° coverage.

    Config params:
        rotation_degrees: How many degrees to rotate (default: 360)
        yaw_rate: Rotation speed in rad/s (overrides mission config)

    State params used:
        scan_yaw_rate: Rotation speed in rad/s
        target_class: Object class to look for
    """

    async def execute(self) -> PhaseResult:
        # Get parameters
        rotation_degrees = self.config.get('rotation_degrees', 360)

        # Auto-reduce rotation if we have front+back cameras
        if len(self.detection.cameras) >= 2 and "back" in self.detection.cameras:
            # Front+back = 360° coverage with 180° rotation
            rotation_degrees = min(rotation_degrees, 180)

        target_rotation = math.radians(rotation_degrees)
        yaw_rate = self.get_param('yaw_rate', self.get_param('scan_yaw_rate', 0.5))
        target_class = self.get_param('target_class', 'orange ball')
        timeout = self.get_param('phase_timeout', 60.0)

        print(f"  Scanning {rotation_degrees}deg")
        print(f"  Cameras: {self.detection.cameras}")
        print(f"  Looking for: '{target_class}'")

        # Clear inventory for fresh scan
        self.state.inventory.clear()

        # Track rotation
        x, y, z, start_yaw = self.drone.pose
        total_rotation = 0.0
        last_yaw = start_yaw
        detections_per_camera = {cam: 0 for cam in self.detection.cameras}

        start_time = asyncio.get_event_loop().time()

        while self.state.running and not self.is_cancelled:
            # Check timeout
            if asyncio.get_event_loop().time() - start_time > timeout:
                self.drone.hover()
                return PhaseResult(
                    status=PhaseStatus.FAILED,
                    message=f"Timeout after {timeout:.0f}s"
                )

            # Check if rotation complete
            if total_rotation >= target_rotation:
                break

            # Rotate
            self.drone.send_velocity(0, 0, 0, yaw_rate)

            # Track rotation progress
            x, y, z, current_yaw = self.drone.pose
            delta_yaw = current_yaw - last_yaw

            # Handle wraparound
            if delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            elif delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi

            total_rotation += abs(delta_yaw)
            last_yaw = current_yaw

            # Process detections from all cameras
            all_detections = self.detection.get_all_detections()
            for camera_id, detection_list in all_detections.items():
                cam_geom = self.detection.get_camera_geometry(camera_id)

                for det in detection_list.detections:
                    # Create tracked object
                    obj = TrackedObject(
                        class_name=det.class_name,
                        confidence=det.confidence,
                        bearing_x=det.bearing_x,
                        bearing_y=det.bearing_y,
                        area=det.area,
                        camera_id=camera_id,
                        drone_x=x,
                        drone_y=y,
                        drone_z=z,
                        drone_yaw=current_yaw,
                        timestamp=time.time()
                    )

                    # Estimate world position
                    obj.estimate_world_position(
                        camera_fov_deg=cam_geom.get("fov", 90.0),
                        camera_pitch_deg=cam_geom["orientation"][1],
                        camera_yaw_deg=cam_geom["orientation"][2]
                    )

                    # Add to inventory (will merge with nearby duplicates)
                    self.state.inventory.add(obj)
                    detections_per_camera[camera_id] += 1

            # Progress update
            progress = total_rotation / target_rotation * 100
            det_str = " ".join(f"{k}:{v}" for k, v in detections_per_camera.items())
            print(f"  Scan: {progress:.0f}% | detections: {det_str}", end='\r')

            await asyncio.sleep(0.05)

        self.drone.hover()
        print()  # Clear progress line

        # Print inventory
        self.state.inventory.print_inventory()

        total_detections = sum(detections_per_camera.values())
        unique_objects = self.state.inventory.count()
        target_count = self.state.inventory.count(target_class)

        return PhaseResult(
            status=PhaseStatus.COMPLETED,
            message=f"Found {unique_objects} objects ({target_count} '{target_class}')",
            data={
                'detections_per_camera': detections_per_camera,
                'total_detections': total_detections,
                'unique_objects': unique_objects,
                'target_count': target_count,
            }
        )
