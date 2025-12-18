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
    orange_score: float   # Color quality (0-1, higher = more orange)
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
        """Get the most ball-like ORANGE object."""
        if not self.objects:
            return None

        # Filter for ball candidates: must have high circularity (> 0.74)
        # A sphere should have circularity close to 1.0
        # A rectangle has circularity ~0.78 or lower (but can appear higher at angles)
        ball_candidates = [o for o in self.objects if o.circularity > 0.74]

        if not ball_candidates:
            print("  WARNING: No objects with circularity > 0.74 (sphere-like)")
            # Fall back to all objects but still prioritize circularity
            ball_candidates = self.objects

        # Score: circularity⁴ × orange_score × √area
        # Power of 4 HEAVILY penalizes non-circular shapes
        # circ=0.73 → 0.28, circ=0.76 → 0.33, circ=0.85 → 0.52, circ=0.95 → 0.81
        return max(ball_candidates, key=lambda o: (o.circularity ** 4) * o.orange_score * math.sqrt(o.area))

    def print_inventory(self):
        """Print all detected objects with scores."""
        print(f"\n  Found {len(self.objects)} object(s):")
        print(f"  (Ball candidates need circularity > 0.74, score uses circ⁴)")
        for i, obj in enumerate(self.objects):
            # Score uses circularity⁴ to heavily penalize non-circular shapes
            score = (obj.circularity ** 4) * obj.orange_score * math.sqrt(obj.area)
            is_candidate = "✓" if obj.circularity > 0.74 else " "
            print(f"    {is_candidate} {i+1}. pos=({obj.world_x:.1f}, {obj.world_y:.1f}) "
                  f"area={obj.area} circ={obj.circularity:.2f} orange={obj.orange_score:.2f} score={score:.1f}")


class OrangeBallDetector:
    """Detects orange ball-shaped objects using HSV color thresholding."""

    def __init__(self):
        # HSV range for orange: Hue 5-25, high saturation, high value
        self.hsv_lower = np.array([5, 120, 120])
        self.hsv_upper = np.array([25, 255, 255])
        self.min_area = 200
        self.min_circularity = 0.4

        # Ideal orange color (Hue ~15, high sat, high val)
        self.ideal_hue = 15.0
        self.ideal_saturation = 200.0
        self.ideal_value = 200.0

    def _circularity(self, contour) -> float:
        """Calculate circularity: 4*pi*area/perimeter^2."""
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return 0
        return 4 * math.pi * area / (perimeter * perimeter)

    def _orange_score(self, hsv_image: np.ndarray, mask: np.ndarray, contour) -> float:
        """
        Calculate how 'orange' the detected region is (0-1).
        Based on how close the mean HSV is to ideal orange.
        """
        # Create mask for just this contour
        contour_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, -1)

        # Get mean HSV values in the contour
        mean_hsv = cv2.mean(hsv_image, mask=contour_mask)
        hue, sat, val = mean_hsv[0], mean_hsv[1], mean_hsv[2]

        # Score based on distance from ideal orange
        # Hue score: ideal is 15, range is 0-180 in OpenCV
        hue_diff = abs(hue - self.ideal_hue)
        hue_score = max(0, 1.0 - hue_diff / 15.0)  # Full score within ±15

        # Saturation score: higher is better for vivid orange
        sat_score = min(1.0, sat / self.ideal_saturation)

        # Value score: higher is better (not too dark)
        val_score = min(1.0, val / self.ideal_value)

        # Combined score (weight hue most heavily)
        return hue_score * 0.5 + sat_score * 0.3 + val_score * 0.2

    def detect_all(self, image: np.ndarray) -> List[Tuple[float, float, int, float, float]]:
        """
        Detect all orange ball-like objects in image.
        Returns list of (bearing_x, bearing_y, area, circularity, orange_score).
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

            # Calculate orange score
            orange_score = self._orange_score(hsv, mask, contour)

            # Get centroid
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Convert to bearing (-1 to 1)
            bearing_x = (cx - width / 2) / (width / 2)
            bearing_y = (cy - height / 2) / (height / 2)

            detections.append((bearing_x, bearing_y, int(area), circularity, orange_score))

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
        """
        Send velocity command.

        IMPORTANT: The AirSim bridge interprets velocities as WORLD FRAME:
          vx = v_north (North component, NED +X)
          vy = v_east (East component, NED +Y)
          vz = v_down (Down component, NED +Z, positive = descend)
        """
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
                for bearing_x, bearing_y, area, circularity, orange_score in detections:
                    obj = DetectedObject(
                        timestamp=time.time(),
                        bearing_x=bearing_x,
                        bearing_y=bearing_y,
                        area=area,
                        circularity=circularity,
                        orange_score=orange_score,
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
            score = (target.circularity ** 4) * target.orange_score * math.sqrt(target.area)
            print(f"\n  SELECTED TARGET:")
            print(f"    World position: ({target.world_x:.1f}, {target.world_y:.1f})")
            print(f"    Detected area: {target.area} pixels")
            print(f"    Circularity: {target.circularity:.2f} (1.0 = perfect circle)")
            print(f"    Orange score: {target.orange_score:.2f} (1.0 = ideal orange)")
            print(f"    Combined score: {score:.1f} (circ⁴ × orange × √area)")
            print(f"    Drone was at: ({target.drone_x:.1f}, {target.drone_y:.1f}, alt={-target.drone_z:.1f}m)")
            print(f"    Drone yaw was: {math.degrees(target.drone_yaw):.0f}°")
            return target
        else:
            print("  No suitable target found!")
            return None

    def phase4_navigate(self, target: DetectedObject) -> bool:
        """
        Phase 4: Navigate to target using "fly over, then descend" strategy.

        This avoids obstacles by:
        1. CRUISE: Fly horizontally at search altitude until above target
        2. DESCEND: Descend vertically to approach altitude
        3. FINAL: Slow approach to target
        """
        print(f"\n[PHASE 4] Navigating to target...")

        target_z = self.approach_altitude  # e.g., -5.0 (5m altitude in NED)
        cruise_z = self.observation_altitude  # Stay at search altitude during cruise
        descent_threshold = 10.0  # Start descent when within this XY distance

        print(f"  Target position: ({target.world_x:.1f}, {target.world_y:.1f})")
        print(f"  Cruise altitude: {-cruise_z:.1f}m (until within {descent_threshold:.0f}m)")
        print(f"  Final altitude: {-target_z:.1f}m")

        x, y, z, yaw = self.get_pose()
        dx = target.world_x - x
        dy = target.world_y - y
        initial_dist = math.sqrt(dx*dx + dy*dy)
        initial_heading = math.atan2(dy, dx)
        print(f"  Current position: ({x:.1f}, {y:.1f}, alt={-z:.1f}m)")
        print(f"  Initial distance: {initial_dist:.1f}m")
        print(f"  Heading to target: {math.degrees(initial_heading):.0f}°")
        print(f"  Current yaw: {math.degrees(yaw):.0f}°")

        nav_speed = 5.0  # m/s horizontal
        descent_speed = 2.0  # m/s vertical

        start_time = time.time()
        last_print = 0

        while self.running and (time.time() - start_time) < 90.0:
            x, y, z, yaw = self.get_pose()

            # Vector to target in world frame (NED: x=North, y=East)
            dx = target.world_x - x
            dy = target.world_y - y
            dist_xy = math.sqrt(dx*dx + dy*dy)
            target_heading = math.atan2(dy, dx)

            # Yaw error (how much we need to turn to face target)
            yaw_error = target_heading - yaw
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi

            target_hdg_deg = math.degrees(target_heading)
            yaw_deg = math.degrees(yaw)
            yaw_err_deg = math.degrees(yaw_error)

            # Determine phase based on position
            at_cruise_alt = z < cruise_z + 2.0  # Within 2m of cruise altitude
            at_final_alt = z > target_z - 1.0   # Within 1m of final altitude
            over_target = dist_xy < descent_threshold

            if dist_xy < 5.0 and at_final_alt:
                # SUCCESS: Close enough horizontally and at correct altitude
                self.hover()
                print(f"\n  ARRIVED! Distance: {dist_xy:.1f}m")
                print(f"  Final position: ({x:.1f}, {y:.1f}, alt={-z:.1f}m)")
                print(f"  Facing: {yaw_deg:.0f}° (target was at {target_hdg_deg:.0f}°)")
                return True

            elif over_target and not at_final_alt:
                # DESCEND: Over target, descend to final altitude
                phase = "DESCEND"
                # Descend while maintaining position (small corrections only)
                vz = descent_speed  # Positive = descend in NED
                # Small position corrections
                correction_speed = min(1.0, dist_xy * 0.2)
                vx_world = correction_speed * math.cos(target_heading)
                vy_world = correction_speed * math.sin(target_heading)
                # Keep facing target
                yaw_rate = 0.5 * yaw_error
                self.send_velocity(vx_world, vy_world, vz, yaw_rate)
                status = f"descending to {-target_z:.0f}m"

            elif over_target and at_final_alt:
                # FINAL: At altitude, close in slowly
                phase = "FINAL"
                approach_speed = min(2.0, dist_xy * 0.3)
                vx_world = approach_speed * math.cos(target_heading)
                vy_world = approach_speed * math.sin(target_heading)
                yaw_rate = 0.5 * yaw_error
                self.send_velocity(vx_world, vy_world, 0, yaw_rate)
                status = f"approaching, spd={approach_speed:.1f}"

            else:
                # CRUISE: Fly horizontally at cruise altitude toward target
                phase = "CRUISE"
                speed = min(nav_speed, max(2.0, dist_xy * 0.15))
                vx_world = speed * math.cos(target_heading)
                vy_world = speed * math.sin(target_heading)

                # Maintain cruise altitude
                if z > cruise_z + 1.0:
                    vz = -descent_speed  # Ascend (negative vz)
                elif z < cruise_z - 1.0:
                    vz = descent_speed   # Descend (positive vz)
                else:
                    vz = 0

                yaw_rate = 1.5 * yaw_error
                self.send_velocity(vx_world, vy_world, vz, yaw_rate)
                status = f"v_n={vx_world:.1f} v_e={vy_world:.1f}"

            # Status update
            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                print(f"  [{phase}] dist={dist_xy:.1f}m alt={-z:.1f}m yaw={yaw_deg:.0f}° {status}")

            time.sleep(0.05)

        self.hover()
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
