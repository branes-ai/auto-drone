#!/usr/bin/env python3
"""
Fly to Orange Ball - Phased Mission

A structured approach to finding and navigating to an orange ball:
  Phase 1: Ascend to observation altitude
  Phase 2: 360° scan - rotate in place, detect objects, estimate world positions
  Phase 3: Inventory - catalog found objects, select orange ball
  Phase 4: Navigate - fly directly to estimated waypoint

Usage:
    python fly_to_orange_ball_phased.py --connect tcp/192.168.1.10:7447
"""

import argparse
import sys
import time
import threading
import math
from typing import Optional, List, Tuple
from dataclasses import dataclass, field

import numpy as np
import cv2
import zenoh

from data_types import (
    ImageData, Odometry, VelocityCommand, Waypoint, WaypointList,
    CommandPriority, CommandSource
)


@dataclass
class DetectedObject:
    """A detected object with estimated world position."""
    timestamp: float
    bearing_x: float      # Image bearing (-1 to 1)
    bearing_y: float
    area: int             # Pixel area
    circularity: float    # Shape metric (1.0 = circle)
    drone_x: float        # Drone position when detected
    drone_y: float
    drone_z: float
    drone_yaw: float      # Drone heading when detected

    # Estimated world position
    world_x: float = 0.0
    world_y: float = 0.0
    world_z: float = 0.0

    def estimate_world_position(self, camera_fov_deg: float = 90.0, camera_pitch_deg: float = -15.0):
        """Estimate world position from bearing and drone pose."""
        # Estimate distance from apparent size (larger = closer)
        # Rough calibration: area of 5000 pixels ≈ 20m away for a 1m ball
        estimated_distance = max(10.0, 100.0 / math.sqrt(self.area / 1000.0 + 1))

        # Camera pitch offset (camera tilted down)
        camera_pitch_rad = math.radians(camera_pitch_deg)

        # Horizontal angle from bearing
        half_fov = math.radians(camera_fov_deg / 2)
        horizontal_angle = self.bearing_x * half_fov

        # Vertical angle from bearing (adjusted for camera pitch)
        vertical_angle = self.bearing_y * half_fov + camera_pitch_rad

        # World heading to target
        target_heading = self.drone_yaw + horizontal_angle

        # Horizontal distance (accounting for vertical angle)
        horizontal_dist = estimated_distance * math.cos(vertical_angle)

        # World position estimate
        self.world_x = self.drone_x + horizontal_dist * math.cos(target_heading)
        self.world_y = self.drone_y + horizontal_dist * math.sin(target_heading)
        # Target is on ground (z=0) or estimate from vertical angle
        self.world_z = 0.0  # Assume ground level


@dataclass
class ObjectInventory:
    """Catalog of detected objects from scan."""
    objects: List[DetectedObject] = field(default_factory=list)

    def add(self, obj: DetectedObject):
        """Add object, merging with nearby existing detections."""
        # Check if this is a re-detection of existing object
        for existing in self.objects:
            dx = obj.world_x - existing.world_x
            dy = obj.world_y - existing.world_y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < 10.0:  # Within 10m = same object
                # Update with better detection (larger area = closer/clearer)
                if obj.area > existing.area:
                    existing.world_x = obj.world_x
                    existing.world_y = obj.world_y
                    existing.area = obj.area
                    existing.circularity = obj.circularity
                return
        # New object
        self.objects.append(obj)

    def get_best_ball(self) -> Optional[DetectedObject]:
        """Get the most ball-like object (highest circularity * area score)."""
        if not self.objects:
            return None
        # Score by circularity and size
        return max(self.objects, key=lambda o: o.circularity * math.sqrt(o.area))

    def print_inventory(self):
        """Print all detected objects."""
        print(f"\n  Found {len(self.objects)} object(s):")
        for i, obj in enumerate(self.objects):
            print(f"    {i+1}. pos=({obj.world_x:.1f}, {obj.world_y:.1f}) "
                  f"area={obj.area} circ={obj.circularity:.2f}")


class OrangeBallDetector:
    """Detects orange ball-shaped objects using HSV color thresholding."""

    def __init__(self):
        self.hsv_lower = np.array([5, 100, 100])
        self.hsv_upper = np.array([25, 255, 255])
        self.min_area = 200
        self.min_circularity = 0.4

    def _circularity(self, contour) -> float:
        """Calculate circularity: 4*pi*area/perimeter^2."""
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return 0
        return 4 * math.pi * area / (perimeter * perimeter)

    def detect_all(self, image: np.ndarray) -> List[Tuple[float, float, int, float]]:
        """
        Detect all orange ball-like objects in image.
        Returns list of (bearing_x, bearing_y, area, circularity).
        """
        if image is None or image.size == 0:
            return []

        height, width = image.shape[:2]

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # Morphological cleanup
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue

            circularity = self._circularity(contour)
            if circularity < self.min_circularity:
                continue

            # Get centroid
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Convert to bearing (-1 to 1)
            bearing_x = (cx - width / 2) / (width / 2)
            bearing_y = (cy - height / 2) / (height / 2)

            detections.append((bearing_x, bearing_y, int(area), circularity))

        return detections


class PhasedMission:
    """Phased mission to find and fly to orange ball."""

    def __init__(self, connect_endpoint: str, robot_id: str = "drone"):
        self.connect_endpoint = connect_endpoint
        self.robot_id = robot_id
        self.running = False
        self.session: Optional[zenoh.Session] = None

        # State
        self.latest_image: Optional[np.ndarray] = None
        self.latest_odom: Optional[Odometry] = None
        self.image_lock = threading.Lock()
        self.odom_lock = threading.Lock()

        # Detector and inventory
        self.detector = OrangeBallDetector()
        self.inventory = ObjectInventory()

        # Topics
        self.topic_rgb = f"robot/{robot_id}/sensor/camera/rgb"
        self.topic_odom = f"robot/{robot_id}/sensor/state/odom"
        self.topic_cmd_vel = f"robot/{robot_id}/cmd/velocity"
        self.topic_waypoints = f"robot/{robot_id}/cmd/waypoints"

        # Parameters
        self.observation_altitude = -25.0  # NED (negative = up)
        self.scan_yaw_rate = 0.5           # rad/s during scan
        self.ascent_speed = 3.0            # m/s
        self.approach_altitude = -5.0      # Final approach altitude

    def connect(self) -> bool:
        """Connect to Zenoh."""
        try:
            print(f"Connecting to Zenoh at {self.connect_endpoint}...")
            config = zenoh.Config()
            config.insert_json5("connect/endpoints", f'["{self.connect_endpoint}"]')
            self.session = zenoh.open(config)
            print("Connected to Zenoh successfully")

            self.session.declare_subscriber(self.topic_rgb, self._on_rgb)
            self.session.declare_subscriber(self.topic_odom, self._on_odom)
            print(f"Subscribed to: {self.topic_rgb}, {self.topic_odom}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def _on_rgb(self, sample: zenoh.Sample):
        try:
            img_data = ImageData.deserialize(bytes(sample.payload))
            with self.image_lock:
                self.latest_image = img_data.to_numpy()
        except:
            pass

    def _on_odom(self, sample: zenoh.Sample):
        try:
            odom = Odometry.deserialize(bytes(sample.payload))
            with self.odom_lock:
                self.latest_odom = odom
        except:
            pass

    def get_image(self) -> Optional[np.ndarray]:
        with self.image_lock:
            return self.latest_image.copy() if self.latest_image is not None else None

    def get_pose(self) -> Tuple[float, float, float, float]:
        """Get current pose (x, y, z, yaw)."""
        with self.odom_lock:
            if self.latest_odom:
                return (self.latest_odom.x, self.latest_odom.y,
                        self.latest_odom.z, self.latest_odom.yaw)
        return (0, 0, 0, 0)

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float):
        cmd = VelocityCommand(
            vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate,
            priority=CommandPriority.NORMAL,
            source=CommandSource.MANUAL_CONTROL,
            timestamp_us=int(time.time() * 1_000_000)
        )
        self.session.put(self.topic_cmd_vel, cmd.serialize())

    def hover(self):
        self.send_velocity(0, 0, 0, 0)

    def send_waypoint(self, x: float, y: float, z: float, speed: float = 3.0):
        """Send a single waypoint."""
        wp = Waypoint(x=x, y=y, z=z, yaw=0.0, speed=speed,
                      flags=Waypoint.FINAL, id=1)
        wpl = WaypointList([wp], timestamp_us=int(time.time() * 1_000_000))
        self.session.put(self.topic_waypoints, wpl.serialize())

    def wait_for_sensors(self, timeout: float = 5.0) -> bool:
        """Wait for camera and odometry."""
        print("Waiting for sensors...")
        start = time.time()
        while time.time() - start < timeout:
            if self.latest_image is not None and self.latest_odom is not None:
                return True
            time.sleep(0.1)
        return False

    def phase1_ascend(self) -> bool:
        """Phase 1: Ascend to observation altitude."""
        print(f"\n[PHASE 1] Ascending to observation altitude ({-self.observation_altitude:.0f}m)...")

        while self.running:
            x, y, z, yaw = self.get_pose()
            altitude = -z

            if z <= self.observation_altitude + 0.5:
                self.hover()
                print(f"  Reached {altitude:.1f}m")
                return True

            self.send_velocity(0, 0, -self.ascent_speed, 0)
            print(f"  Altitude: {altitude:.1f}m", end='\r')
            time.sleep(0.1)

        return False

    def phase2_scan(self) -> bool:
        """Phase 2: 360° scan to detect objects."""
        print(f"\n[PHASE 2] Scanning 360°...")

        x, y, z, start_yaw = self.get_pose()
        total_rotation = 0.0
        last_yaw = start_yaw
        scan_start = time.time()
        detections_count = 0

        while self.running and total_rotation < 2 * math.pi:
            # Rotate
            self.send_velocity(0, 0, 0, self.scan_yaw_rate)

            # Track rotation
            x, y, z, current_yaw = self.get_pose()
            delta_yaw = current_yaw - last_yaw
            # Handle wrap-around
            if delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            elif delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi
            total_rotation += abs(delta_yaw)
            last_yaw = current_yaw

            # Detect objects
            image = self.get_image()
            if image is not None:
                detections = self.detector.detect_all(image)
                for bearing_x, bearing_y, area, circularity in detections:
                    obj = DetectedObject(
                        timestamp=time.time(),
                        bearing_x=bearing_x,
                        bearing_y=bearing_y,
                        area=area,
                        circularity=circularity,
                        drone_x=x, drone_y=y, drone_z=z,
                        drone_yaw=current_yaw
                    )
                    obj.estimate_world_position()
                    self.inventory.add(obj)
                    detections_count += 1

            progress = total_rotation / (2 * math.pi) * 100
            print(f"  Scan: {progress:.0f}% ({detections_count} detections)", end='\r')
            time.sleep(0.05)

        self.hover()
        scan_time = time.time() - scan_start
        print(f"\n  Scan complete in {scan_time:.1f}s")
        return True

    def phase3_inventory(self) -> Optional[DetectedObject]:
        """Phase 3: Review inventory and select target."""
        print(f"\n[PHASE 3] Reviewing inventory...")

        self.inventory.print_inventory()

        target = self.inventory.get_best_ball()
        if target:
            print(f"\n  Selected target: ({target.world_x:.1f}, {target.world_y:.1f})")
            return target
        else:
            print("  No suitable target found!")
            return None

    def phase4_navigate(self, target: DetectedObject) -> bool:
        """Phase 4: Navigate to target location."""
        print(f"\n[PHASE 4] Navigating to target...")

        # Send waypoint at approach altitude
        target_z = self.approach_altitude
        print(f"  Waypoint: ({target.world_x:.1f}, {target.world_y:.1f}, {-target_z:.1f}m)")
        self.send_waypoint(target.world_x, target.world_y, target_z, speed=5.0)

        # Monitor progress
        start_time = time.time()
        while self.running and (time.time() - start_time) < 60.0:
            x, y, z, yaw = self.get_pose()
            dx = target.world_x - x
            dy = target.world_y - y
            dist = math.sqrt(dx*dx + dy*dy)

            print(f"  Distance: {dist:.1f}m  pos=({x:.1f}, {y:.1f}, {-z:.1f}m)", end='\r')

            if dist < 3.0:
                print(f"\n  Arrived at target!")
                return True

            time.sleep(0.2)

        print(f"\n  Navigation timeout")
        return False

    def run(self, timeout: float = 120.0):
        """Run the phased mission."""
        self.running = True

        print("\n" + "=" * 60)
        print("Fly to Orange Ball - Phased Mission")
        print("=" * 60)
        print(f"  Observation altitude: {-self.observation_altitude:.0f}m")
        print(f"  Approach altitude: {-self.approach_altitude:.0f}m")
        print(f"  Timeout: {timeout}s")
        print("=" * 60)

        if not self.wait_for_sensors():
            print("ERROR: Sensors not available!")
            return

        x, y, z, yaw = self.get_pose()
        print(f"\nStarting pose: ({x:.1f}, {y:.1f}, {-z:.1f}m) yaw={math.degrees(yaw):.0f}°")

        start_time = time.time()

        # Phase 1: Ascend
        if not self.phase1_ascend():
            return

        # Phase 2: Scan
        if not self.phase2_scan():
            return

        # Phase 3: Inventory
        target = self.phase3_inventory()
        if not target:
            print("\nMission failed: No target found")
            return

        # Phase 4: Navigate
        success = self.phase4_navigate(target)

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
        self.running = False

    def disconnect(self):
        if self.session:
            self.session.close()
            self.session = None


def main():
    parser = argparse.ArgumentParser(description="Phased mission to fly to orange ball")
    parser.add_argument("--connect", required=True, help="Zenoh endpoint")
    parser.add_argument("--robot-id", default="drone", help="Robot ID")
    parser.add_argument("--altitude", type=float, default=25.0, help="Observation altitude (m)")
    parser.add_argument("--approach", type=float, default=5.0, help="Approach altitude (m)")
    parser.add_argument("--timeout", type=float, default=120.0, help="Mission timeout (s)")

    args = parser.parse_args()

    mission = PhasedMission(args.connect, args.robot_id)
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
