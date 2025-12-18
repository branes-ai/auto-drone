#!/usr/bin/env python3
"""
Fly to Orange Ball - Multi-Camera Mission

Leverages front, back, and down cameras for efficient target acquisition
and precision approach.

Phases:
  1. ASCEND - Rise to observation altitude
  2. SCAN - 180° rotation (front+back = full 360° coverage)
  3. SELECT - Choose best target from combined detections
  4. CRUISE - Fly to target at safe altitude
  5. DESCEND - Drop to approach altitude when overhead
  6. APPROACH - Final approach using front OR down camera

Camera Roles:
  - Front: Primary navigation, target tracking during approach
  - Back:  Simultaneous scanning during 180° rotation, rear awareness
  - Down:  Terminal precision when directly over target

Usage:
    # Terminal 1: Bridge
    python airsim_zenoh_bridge.py --connect tcp/localhost:7447

    # Terminal 2: Multi-camera detector
    python yolo_multi_camera_detector.py --connect tcp/localhost:7447 \\
        --cameras front back down --prompts "orange ball"

    # Terminal 3: This mission
    python fly_to_orange_ball_multicam.py --connect tcp/localhost:7447
"""

import argparse
import sys
import time
import threading
import math
from typing import Optional, List, Dict, Tuple
from dataclasses import dataclass, field

import zenoh

from data_types import (
    Odometry, VelocityCommand, Detection, DetectionList, CameraDetectionList,
    CommandPriority, CommandSource
)


# Camera geometry from robot_quadrotor_vision.jsonc
CAMERA_GEOMETRY = {
    "front": {"position": (0.30, 0.0, -0.05), "orientation": (0.0, -15.0, 0.0)},
    "back":  {"position": (-0.15, 0.0, 0.0), "orientation": (0.0, 0.0, 180.0)},
    "down":  {"position": (0.0, 0.0, 0.05), "orientation": (0.0, -90.0, 0.0)},
}


@dataclass
class TrackedObject:
    """An object detected during scan with estimated world position."""
    class_name: str
    confidence: float
    bearing_x: float
    bearing_y: float
    area: int
    camera_id: str          # Which camera saw it
    drone_x: float
    drone_y: float
    drone_z: float
    drone_yaw: float
    timestamp: float

    # Estimated world position
    world_x: float = 0.0
    world_y: float = 0.0

    def estimate_world_position(self):
        """Estimate world position from bearing and drone pose."""
        # Get camera orientation
        cam_geom = CAMERA_GEOMETRY.get(self.camera_id, CAMERA_GEOMETRY["front"])
        camera_pitch_deg = cam_geom["orientation"][1]
        camera_yaw_deg = cam_geom["orientation"][2]

        # Estimate distance from apparent size
        estimated_distance = max(10.0, 100.0 / math.sqrt(self.area / 1000.0 + 1))

        # Camera angles
        camera_pitch_rad = math.radians(camera_pitch_deg)
        camera_yaw_rad = math.radians(camera_yaw_deg)
        half_fov = math.radians(45.0)  # 90° FOV / 2

        # Horizontal angle from bearing (adjusted for camera yaw)
        horizontal_angle = self.bearing_x * half_fov + camera_yaw_rad

        # Vertical angle from bearing (adjusted for camera pitch)
        vertical_angle = self.bearing_y * half_fov + camera_pitch_rad

        # World heading to target
        target_heading = self.drone_yaw + horizontal_angle

        # Horizontal distance
        horizontal_dist = estimated_distance * math.cos(vertical_angle)

        # World position
        self.world_x = self.drone_x + horizontal_dist * math.cos(target_heading)
        self.world_y = self.drone_y + horizontal_dist * math.sin(target_heading)


@dataclass
class ObjectInventory:
    """Catalog of detected objects from all cameras."""
    objects: List[TrackedObject] = field(default_factory=list)

    def add(self, obj: TrackedObject):
        """Add object, merging with nearby existing detections."""
        for existing in self.objects:
            if existing.class_name != obj.class_name:
                continue
            dx = obj.world_x - existing.world_x
            dy = obj.world_y - existing.world_y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < 10.0:  # Same object
                # Update if better confidence
                if obj.confidence > existing.confidence:
                    existing.world_x = obj.world_x
                    existing.world_y = obj.world_y
                    existing.confidence = obj.confidence
                    existing.area = obj.area
                    existing.camera_id = obj.camera_id
                return
        self.objects.append(obj)

    def get_best(self, class_name: str) -> Optional[TrackedObject]:
        """Get highest confidence object of given class."""
        matches = [o for o in self.objects if o.class_name.lower() == class_name.lower()]
        return max(matches, key=lambda o: o.confidence) if matches else None

    def print_inventory(self):
        """Print all detected objects."""
        print(f"\n  Found {len(self.objects)} object(s):")
        for i, obj in enumerate(self.objects):
            print(f"    {i+1}. [{obj.camera_id}] {obj.class_name}: conf={obj.confidence:.2f} "
                  f"pos=({obj.world_x:.1f}, {obj.world_y:.1f}) area={obj.area}")


class MultiCameraMission:
    """Mission using multiple cameras for efficient target acquisition."""

    def __init__(self, connect_endpoint: str, robot_id: str = "drone"):
        self.connect_endpoint = connect_endpoint
        self.robot_id = robot_id
        self.running = False
        self.session: Optional[zenoh.Session] = None

        # Per-camera detection state
        self.camera_detections: Dict[str, Optional[CameraDetectionList]] = {
            "front": None, "back": None, "down": None
        }
        self.detection_locks: Dict[str, threading.Lock] = {
            "front": threading.Lock(),
            "back": threading.Lock(),
            "down": threading.Lock(),
        }

        # Odometry
        self.latest_odom: Optional[Odometry] = None
        self.odom_lock = threading.Lock()

        # Inventory
        self.inventory = ObjectInventory()

        # Topics
        self.topic_odom = f"robot/{robot_id}/sensor/state/odom"
        self.topic_cmd_vel = f"robot/{robot_id}/cmd/velocity"

        # Parameters
        self.target_class = "orange ball"
        self.observation_altitude = -25.0  # NED
        self.approach_altitude = -5.0
        self.scan_yaw_rate = 0.5  # rad/s
        self.ascent_speed = 3.0
        self.nav_speed = 5.0
        self.descent_speed = 2.0

    def connect(self) -> bool:
        """Connect to Zenoh and subscribe to topics."""
        try:
            print(f"Connecting to Zenoh at {self.connect_endpoint}...")
            config = zenoh.Config()
            config.insert_json5("connect/endpoints", f'["{self.connect_endpoint}"]')
            self.session = zenoh.open(config)
            print("Connected to Zenoh successfully")

            # Subscribe to per-camera detections
            for camera_id in ["front", "back", "down"]:
                topic = f"robot/{self.robot_id}/perception/detections/{camera_id}"
                self.session.declare_subscriber(
                    topic,
                    lambda sample, cid=camera_id: self._on_detection(sample, cid)
                )
                print(f"  Subscribed to: {topic}")

            # Subscribe to odometry
            self.session.declare_subscriber(self.topic_odom, self._on_odom)
            print(f"  Subscribed to: {self.topic_odom}")

            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def _on_detection(self, sample: zenoh.Sample, camera_id: str):
        """Handle detection from a specific camera."""
        try:
            detections = CameraDetectionList.deserialize(bytes(sample.payload))
            with self.detection_locks[camera_id]:
                self.camera_detections[camera_id] = detections
        except Exception as e:
            # Try legacy format
            try:
                detections = DetectionList.deserialize(bytes(sample.payload))
                # Convert to CameraDetectionList
                cam_det = CameraDetectionList(
                    timestamp_us=detections.timestamp_us,
                    frame_id=detections.frame_id,
                    image_width=detections.image_width,
                    image_height=detections.image_height,
                    inference_time_ms=detections.inference_time_ms,
                    camera_id=camera_id,
                    detections=detections.detections
                )
                with self.detection_locks[camera_id]:
                    self.camera_detections[camera_id] = cam_det
            except:
                pass

    def _on_odom(self, sample: zenoh.Sample):
        """Handle odometry updates."""
        try:
            odom = Odometry.deserialize(bytes(sample.payload))
            with self.odom_lock:
                self.latest_odom = odom
        except:
            pass

    def get_detections(self, camera_id: str) -> Optional[CameraDetectionList]:
        """Get latest detections from a camera."""
        with self.detection_locks[camera_id]:
            return self.camera_detections[camera_id]

    def get_all_detections(self) -> Dict[str, CameraDetectionList]:
        """Get latest detections from all cameras."""
        result = {}
        for camera_id in ["front", "back", "down"]:
            det = self.get_detections(camera_id)
            if det:
                result[camera_id] = det
        return result

    def get_pose(self) -> Tuple[float, float, float, float]:
        """Get current pose (x, y, z, yaw)."""
        with self.odom_lock:
            if self.latest_odom:
                return (self.latest_odom.x, self.latest_odom.y,
                        self.latest_odom.z, self.latest_odom.yaw)
        return (0, 0, 0, 0)

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """Send velocity command (world frame: vx=north, vy=east, vz=down)."""
        cmd = VelocityCommand(
            vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate,
            priority=CommandPriority.NORMAL,
            source=CommandSource.MANUAL_CONTROL,
            timestamp_us=int(time.time() * 1_000_000)
        )
        self.session.put(self.topic_cmd_vel, cmd.serialize())

    def hover(self):
        """Send hover command."""
        self.send_velocity(0, 0, 0, 0)

    def wait_for_data(self, timeout: float = 10.0) -> bool:
        """Wait for detections and odometry."""
        print("Waiting for detector and odometry...")
        start = time.time()
        while time.time() - start < timeout:
            has_detections = any(self.camera_detections.values())
            has_odom = self.latest_odom is not None
            if has_detections and has_odom:
                return True
            time.sleep(0.1)
        return False

    # =========================================================================
    # Mission Phases
    # =========================================================================

    def phase1_ascend(self) -> bool:
        """Phase 1: Ascend to observation altitude."""
        print(f"\n[PHASE 1] Ascending to {-self.observation_altitude:.0f}m...")

        while self.running:
            x, y, z, yaw = self.get_pose()

            if z <= self.observation_altitude + 0.5:
                self.hover()
                print(f"  Reached {-z:.1f}m")
                return True

            self.send_velocity(0, 0, -self.ascent_speed, 0)
            print(f"  Altitude: {-z:.1f}m", end='\r')
            time.sleep(0.1)

        return False

    def phase2_scan(self) -> bool:
        """
        Phase 2: 180° scan using front+back cameras.

        With front and back cameras, 180° rotation gives full 360° coverage.
        """
        print(f"\n[PHASE 2] Scanning 180° (front+back = 360° coverage)...")

        x, y, z, start_yaw = self.get_pose()
        total_rotation = 0.0
        last_yaw = start_yaw
        detections_count = {"front": 0, "back": 0, "down": 0}

        # Only need 180° rotation with front+back cameras
        target_rotation = math.pi  # 180°

        while self.running and total_rotation < target_rotation:
            # Rotate
            self.send_velocity(0, 0, 0, self.scan_yaw_rate)

            # Track rotation
            x, y, z, current_yaw = self.get_pose()
            delta_yaw = current_yaw - last_yaw
            if delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            elif delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi
            total_rotation += abs(delta_yaw)
            last_yaw = current_yaw

            # Process detections from ALL cameras
            all_detections = self.get_all_detections()
            for camera_id, detection_list in all_detections.items():
                for det in detection_list.detections:
                    obj = TrackedObject(
                        class_name=det.class_name,
                        confidence=det.confidence,
                        bearing_x=det.bearing_x,
                        bearing_y=det.bearing_y,
                        area=det.area,
                        camera_id=camera_id,
                        drone_x=x, drone_y=y, drone_z=z,
                        drone_yaw=current_yaw,
                        timestamp=time.time()
                    )
                    obj.estimate_world_position()
                    self.inventory.add(obj)
                    detections_count[camera_id] += 1

            progress = total_rotation / target_rotation * 100
            total_det = sum(detections_count.values())
            print(f"  Scan: {progress:.0f}% | detections: F={detections_count['front']} "
                  f"B={detections_count['back']} D={detections_count['down']}", end='\r')
            time.sleep(0.05)

        self.hover()
        print(f"\n  Scan complete (180° = full coverage with front+back)")
        print(f"  Total detections: front={detections_count['front']}, "
              f"back={detections_count['back']}, down={detections_count['down']}")
        return True

    def phase3_select(self) -> Optional[TrackedObject]:
        """Phase 3: Select target from inventory."""
        print(f"\n[PHASE 3] Selecting target...")

        self.inventory.print_inventory()

        target = self.inventory.get_best(self.target_class)
        if target:
            print(f"\n  SELECTED TARGET: {target.class_name}")
            print(f"    Confidence: {target.confidence:.2f}")
            print(f"    Detected by: {target.camera_id} camera")
            print(f"    World position: ({target.world_x:.1f}, {target.world_y:.1f})")
            return target
        else:
            print(f"  No '{self.target_class}' found!")
            return None

    def phase4_cruise(self, target: TrackedObject) -> bool:
        """Phase 4: Cruise to target at safe altitude."""
        print(f"\n[PHASE 4] Cruising to target area...")

        cruise_z = self.observation_altitude
        descent_threshold = 10.0

        print(f"  Target: ({target.world_x:.1f}, {target.world_y:.1f})")
        print(f"  Maintaining altitude: {-cruise_z:.0f}m until within {descent_threshold:.0f}m")

        start_time = time.time()
        last_print = 0

        while self.running and (time.time() - start_time) < 60.0:
            x, y, z, yaw = self.get_pose()

            dx = target.world_x - x
            dy = target.world_y - y
            dist_xy = math.sqrt(dx*dx + dy*dy)
            target_heading = math.atan2(dy, dx)

            # Check if we're over the target
            if dist_xy < descent_threshold:
                self.hover()
                print(f"\n  Over target area (dist={dist_xy:.1f}m)")
                return True

            # Yaw error
            yaw_error = target_heading - yaw
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi

            # Cruise velocity (world frame)
            speed = min(self.nav_speed, max(2.0, dist_xy * 0.15))
            vx_world = speed * math.cos(target_heading)
            vy_world = speed * math.sin(target_heading)

            # Maintain cruise altitude
            if z > cruise_z + 1.0:
                vz = -self.descent_speed
            elif z < cruise_z - 1.0:
                vz = self.descent_speed
            else:
                vz = 0

            yaw_rate = 1.5 * yaw_error
            self.send_velocity(vx_world, vy_world, vz, yaw_rate)

            # Status
            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                print(f"  [CRUISE] dist={dist_xy:.1f}m alt={-z:.1f}m")

            time.sleep(0.05)

        self.hover()
        return False

    def phase5_descend(self, target: TrackedObject) -> bool:
        """Phase 5: Descend to approach altitude."""
        print(f"\n[PHASE 5] Descending to approach altitude...")

        target_z = self.approach_altitude
        print(f"  Target altitude: {-target_z:.0f}m")

        start_time = time.time()
        last_print = 0

        while self.running and (time.time() - start_time) < 30.0:
            x, y, z, yaw = self.get_pose()

            # Check if at approach altitude
            if z > target_z - 1.0:
                self.hover()
                print(f"\n  At approach altitude: {-z:.1f}m")
                return True

            # Descend while maintaining position
            dx = target.world_x - x
            dy = target.world_y - y
            dist_xy = math.sqrt(dx*dx + dy*dy)
            target_heading = math.atan2(dy, dx)

            # Small position corrections
            correction_speed = min(1.0, dist_xy * 0.3)
            vx_world = correction_speed * math.cos(target_heading)
            vy_world = correction_speed * math.sin(target_heading)

            # Yaw toward target
            yaw_error = target_heading - yaw
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi

            self.send_velocity(vx_world, vy_world, self.descent_speed, 0.5 * yaw_error)

            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                print(f"  [DESCEND] alt={-z:.1f}m dist={dist_xy:.1f}m")

            time.sleep(0.05)

        self.hover()
        return False

    def phase6_approach(self, target: TrackedObject) -> bool:
        """
        Phase 6: Final approach using live camera feedback.

        Uses front camera for horizontal approach, switches to down camera
        if target is directly below.
        """
        print(f"\n[PHASE 6] Final approach with visual tracking...")

        start_time = time.time()
        last_print = 0
        active_camera = "front"

        while self.running and (time.time() - start_time) < 60.0:
            x, y, z, yaw = self.get_pose()

            # Get live detections
            front_det = self.get_detections("front")
            down_det = self.get_detections("down")

            # Find target in cameras
            front_target = None
            down_target = None

            if front_det:
                matches = [d for d in front_det.detections
                          if d.class_name.lower() == self.target_class.lower()]
                if matches:
                    front_target = max(matches, key=lambda d: d.confidence)

            if down_det:
                matches = [d for d in down_det.detections
                          if d.class_name.lower() == self.target_class.lower()]
                if matches:
                    down_target = max(matches, key=lambda d: d.confidence)

            # Decide which camera to use
            # Prefer down camera if target is visible and we're close
            dx = target.world_x - x
            dy = target.world_y - y
            dist_xy = math.sqrt(dx*dx + dy*dy)

            if down_target and dist_xy < 8.0:
                active_camera = "down"
                detection = down_target
            elif front_target:
                active_camera = "front"
                detection = front_target
            else:
                # No visual - use waypoint
                active_camera = "waypoint"
                detection = None

            # Check success condition
            if dist_xy < 3.0:
                self.hover()
                print(f"\n  ARRIVED! Distance: {dist_xy:.1f}m")
                print(f"  Final position: ({x:.1f}, {y:.1f}, alt={-z:.1f}m)")
                return True

            # Control based on active camera
            if active_camera == "down" and detection:
                # Down camera: bearing_x/y directly control position
                # Positive bearing_x = target is to the right = move east
                # Positive bearing_y = target is below center = move north (camera faces down)
                vx_world = -detection.bearing_y * 2.0  # Forward/back
                vy_world = detection.bearing_x * 2.0   # Left/right
                vz = 0.5  # Slow descent
                yaw_rate = 0
                status = f"DOWN bearing=({detection.bearing_x:.2f}, {detection.bearing_y:.2f})"

            elif active_camera == "front" and detection:
                # Front camera: bearing_x controls yaw, move forward
                target_heading = math.atan2(dy, dx)
                approach_speed = min(2.0, dist_xy * 0.3)
                vx_world = approach_speed * math.cos(target_heading)
                vy_world = approach_speed * math.sin(target_heading)
                vz = 0
                yaw_rate = detection.bearing_x * 1.0  # Turn toward target
                status = f"FRONT bearing=({detection.bearing_x:.2f}, {detection.bearing_y:.2f})"

            else:
                # Waypoint fallback
                target_heading = math.atan2(dy, dx)
                approach_speed = min(1.5, dist_xy * 0.2)
                vx_world = approach_speed * math.cos(target_heading)
                vy_world = approach_speed * math.sin(target_heading)
                vz = 0
                yaw_error = target_heading - yaw
                while yaw_error > math.pi:
                    yaw_error -= 2 * math.pi
                while yaw_error < -math.pi:
                    yaw_error += 2 * math.pi
                yaw_rate = yaw_error * 0.5
                status = "WAYPOINT (no visual)"

            self.send_velocity(vx_world, vy_world, vz, yaw_rate)

            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                print(f"  [{active_camera.upper()}] dist={dist_xy:.1f}m alt={-z:.1f}m {status}")

            time.sleep(0.05)

        self.hover()
        return False

    def run(self, timeout: float = 180.0):
        """Run the multi-camera mission."""
        self.running = True

        print("\n" + "=" * 60)
        print("Fly to Orange Ball - Multi-Camera Mission")
        print("=" * 60)
        print(f"  Target: '{self.target_class}'")
        print(f"  Cameras: front (nav) + back (scan) + down (precision)")
        print(f"  Observation altitude: {-self.observation_altitude:.0f}m")
        print(f"  Approach altitude: {-self.approach_altitude:.0f}m")
        print(f"  Scan: 180° (front+back = 360° coverage)")
        print("=" * 60)

        if not self.wait_for_data():
            print("ERROR: No detector data! Is yolo_multi_camera_detector.py running?")
            return

        x, y, z, yaw = self.get_pose()
        print(f"\nStarting pose: ({x:.1f}, {y:.1f}, {-z:.1f}m) yaw={math.degrees(yaw):.0f}°")

        start_time = time.time()

        # Execute phases
        if not self.phase1_ascend():
            return

        if not self.phase2_scan():
            return

        target = self.phase3_select()
        if not target:
            print("\nMission failed: No target found")
            return

        if not self.phase4_cruise(target):
            print("\nMission failed: Cruise timeout")
            return

        if not self.phase5_descend(target):
            print("\nMission failed: Descent timeout")
            return

        success = self.phase6_approach(target)

        # Final status
        self.hover()
        elapsed = time.time() - start_time
        x, y, z, yaw = self.get_pose()
        print(f"\n{'=' * 60}")
        print(f"Mission {'SUCCEEDED' if success else 'INCOMPLETE'}")
        print(f"Final position: ({x:.1f}, {y:.1f}, {-z:.1f}m)")
        print(f"Elapsed time: {elapsed:.1f}s")
        print("=" * 60)

    def stop(self):
        """Stop the mission."""
        self.running = False

    def disconnect(self):
        """Close Zenoh session."""
        if self.session:
            self.session.close()
            self.session = None


def main():
    parser = argparse.ArgumentParser(
        description="Multi-camera mission to fly to orange ball",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Prerequisites:
  1. Start AirSim bridge: python airsim_zenoh_bridge.py --connect tcp/localhost:7447
  2. Start multi-camera detector: python yolo_multi_camera_detector.py --connect tcp/localhost:7447 \\
         --cameras front back down --prompts "orange ball"
  3. Run this mission: python fly_to_orange_ball_multicam.py --connect tcp/localhost:7447
        """
    )
    parser.add_argument("--connect", required=True, help="Zenoh endpoint")
    parser.add_argument("--robot-id", default="drone", help="Robot ID")
    parser.add_argument("--target", default="orange ball", help="Target class")
    parser.add_argument("--altitude", type=float, default=25.0, help="Observation altitude (m)")
    parser.add_argument("--approach", type=float, default=5.0, help="Approach altitude (m)")
    parser.add_argument("--timeout", type=float, default=180.0, help="Mission timeout (s)")

    args = parser.parse_args()

    mission = MultiCameraMission(args.connect, args.robot_id)
    mission.target_class = args.target
    mission.observation_altitude = -args.altitude
    mission.approach_altitude = -args.approach

    if not mission.connect():
        return 1

    try:
        mission.run(args.timeout)
    except KeyboardInterrupt:
        print("\nInterrupted.")

    mission.stop()
    mission.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
