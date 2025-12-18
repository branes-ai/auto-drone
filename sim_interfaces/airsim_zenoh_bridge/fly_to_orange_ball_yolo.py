#!/usr/bin/env python3
"""
Fly to Orange Ball Mission - YOLO-World Edition

A simplified mission using YOLO-World for object detection instead of
manual HSV color thresholding and circularity calculations.

Phases:
  1. ASCEND - Ascend to observation altitude
  2. CONFIGURE - Send detection prompts to YOLO detector
  3. SCAN - Rotate 360°, collect detected objects with world position estimates
  4. SELECT - Choose best "orange ball" detection by confidence
  5. NAVIGATE - Fly to target using "fly over, then descend" strategy
  6. APPROACH - Final visual servoing using live detections

Usage:
    # Terminal 1: AirSim bridge
    python airsim_zenoh_bridge.py --connect tcp/localhost:7447

    # Terminal 2: YOLO detector
    python yolo_world_detector.py --connect tcp/localhost:7447 --prompts "orange ball"

    # Terminal 3: Mission
    python fly_to_orange_ball_yolo.py --connect tcp/localhost:7447
"""

import argparse
import sys
import time
import threading
import math
from typing import Optional, List, Tuple
from dataclasses import dataclass, field

import zenoh

from data_types import (
    Odometry, VelocityCommand, Detection, DetectionList, DetectorConfig,
    CommandPriority, CommandSource
)


@dataclass
class TrackedObject:
    """An object detected during scan with estimated world position."""
    class_name: str
    confidence: float
    bearing_x: float
    bearing_y: float
    area: int
    drone_x: float
    drone_y: float
    drone_z: float
    drone_yaw: float
    timestamp: float

    # Estimated world position
    world_x: float = 0.0
    world_y: float = 0.0

    def estimate_world_position(self, camera_fov_deg: float = 90.0, camera_pitch_deg: float = -15.0):
        """Estimate world position from bearing and drone pose."""
        # Estimate distance from apparent size
        estimated_distance = max(10.0, 100.0 / math.sqrt(self.area / 1000.0 + 1))

        # Camera pitch offset
        camera_pitch_rad = math.radians(camera_pitch_deg)
        half_fov = math.radians(camera_fov_deg / 2)

        # Angles
        horizontal_angle = self.bearing_x * half_fov
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
    """Catalog of detected objects from scan."""
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
            print(f"    {i+1}. {obj.class_name}: conf={obj.confidence:.2f} "
                  f"pos=({obj.world_x:.1f}, {obj.world_y:.1f}) area={obj.area}")


class YoloOrangeBallMission:
    """Mission to find and fly to orange ball using YOLO-World detections."""

    def __init__(self, connect_endpoint: str, robot_id: str = "drone"):
        self.connect_endpoint = connect_endpoint
        self.robot_id = robot_id
        self.running = False
        self.session: Optional[zenoh.Session] = None

        # State
        self.latest_detections: Optional[DetectionList] = None
        self.latest_odom: Optional[Odometry] = None
        self.detections_lock = threading.Lock()
        self.odom_lock = threading.Lock()

        # Inventory
        self.inventory = ObjectInventory()

        # Topics
        self.topic_detections = f"robot/{robot_id}/perception/detections"
        self.topic_odom = f"robot/{robot_id}/sensor/state/odom"
        self.topic_cmd_vel = f"robot/{robot_id}/cmd/velocity"
        self.topic_detector_config = f"robot/{robot_id}/detector/config"

        # Parameters
        self.target_class = "orange ball"
        self.observation_altitude = -25.0  # NED (negative = up)
        self.approach_altitude = -5.0
        self.scan_yaw_rate = 0.5  # rad/s
        self.ascent_speed = 3.0
        self.nav_speed = 5.0
        self.descent_speed = 2.0

    def connect(self) -> bool:
        """Connect to Zenoh."""
        try:
            print(f"Connecting to Zenoh at {self.connect_endpoint}...")
            config = zenoh.Config()
            config.insert_json5("connect/endpoints", f'["{self.connect_endpoint}"]')
            self.session = zenoh.open(config)
            print("Connected to Zenoh successfully")

            self.session.declare_subscriber(self.topic_detections, self._on_detections)
            self.session.declare_subscriber(self.topic_odom, self._on_odom)
            print(f"Subscribed to: {self.topic_detections}, {self.topic_odom}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def _on_detections(self, sample: zenoh.Sample):
        """Handle detection updates."""
        try:
            detections = DetectionList.deserialize(bytes(sample.payload))
            with self.detections_lock:
                self.latest_detections = detections
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

    def get_detections(self) -> Optional[DetectionList]:
        """Get latest detections."""
        with self.detections_lock:
            return self.latest_detections

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
        """Send hover command."""
        self.send_velocity(0, 0, 0, 0)

    def configure_detector(self, prompts: List[str]):
        """Send configuration to detector node."""
        config = DetectorConfig(prompts=prompts)
        self.session.put(self.topic_detector_config, config.serialize())
        print(f"  Sent detector config: prompts={prompts}")

    def wait_for_data(self, timeout: float = 10.0) -> bool:
        """Wait for detections and odometry."""
        print("Waiting for detector and odometry...")
        start = time.time()
        while time.time() - start < timeout:
            if self.latest_detections is not None and self.latest_odom is not None:
                return True
            time.sleep(0.1)
        return False

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

    def phase2_configure(self) -> bool:
        """Phase 2: Configure detector with prompts."""
        print(f"\n[PHASE 2] Configuring detector...")

        # Send prompts for what we want to detect
        prompts = [self.target_class]
        self.configure_detector(prompts)

        # Wait a moment for detector to process
        time.sleep(1.0)
        return True

    def phase3_scan(self) -> bool:
        """Phase 3: 360° scan to detect objects."""
        print(f"\n[PHASE 3] Scanning 360° for '{self.target_class}'...")

        x, y, z, start_yaw = self.get_pose()
        total_rotation = 0.0
        last_yaw = start_yaw
        detections_count = 0

        while self.running and total_rotation < 2 * math.pi:
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

            # Process detections
            detection_list = self.get_detections()
            if detection_list:
                for det in detection_list.detections:
                    # Create tracked object
                    obj = TrackedObject(
                        class_name=det.class_name,
                        confidence=det.confidence,
                        bearing_x=det.bearing_x,
                        bearing_y=det.bearing_y,
                        area=det.area,
                        drone_x=x, drone_y=y, drone_z=z,
                        drone_yaw=current_yaw,
                        timestamp=time.time()
                    )
                    obj.estimate_world_position()
                    self.inventory.add(obj)
                    detections_count += 1

            progress = total_rotation / (2 * math.pi) * 100
            print(f"  Scan: {progress:.0f}% ({detections_count} detections)", end='\r')
            time.sleep(0.05)

        self.hover()
        print(f"\n  Scan complete")
        return True

    def phase4_select(self) -> Optional[TrackedObject]:
        """Phase 4: Select target from inventory."""
        print(f"\n[PHASE 4] Selecting target...")

        self.inventory.print_inventory()

        target = self.inventory.get_best(self.target_class)
        if target:
            print(f"\n  SELECTED TARGET: {target.class_name}")
            print(f"    Confidence: {target.confidence:.2f}")
            print(f"    World position: ({target.world_x:.1f}, {target.world_y:.1f})")
            return target
        else:
            print(f"  No '{self.target_class}' found!")
            return None

    def phase5_navigate(self, target: TrackedObject) -> bool:
        """Phase 5: Navigate to target using fly-over-then-descend."""
        print(f"\n[PHASE 5] Navigating to target...")

        target_z = self.approach_altitude
        cruise_z = self.observation_altitude
        descent_threshold = 10.0

        print(f"  Target position: ({target.world_x:.1f}, {target.world_y:.1f})")
        print(f"  Cruise altitude: {-cruise_z:.1f}m")
        print(f"  Final altitude: {-target_z:.1f}m")

        x, y, z, yaw = self.get_pose()
        print(f"  Current position: ({x:.1f}, {y:.1f}, alt={-z:.1f}m)")

        start_time = time.time()
        last_print = 0

        while self.running and (time.time() - start_time) < 90.0:
            x, y, z, yaw = self.get_pose()

            # Vector to target
            dx = target.world_x - x
            dy = target.world_y - y
            dist_xy = math.sqrt(dx*dx + dy*dy)
            target_heading = math.atan2(dy, dx)

            # Yaw error
            yaw_error = target_heading - yaw
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi

            yaw_deg = math.degrees(yaw)
            at_final_alt = z > target_z - 1.0
            over_target = dist_xy < descent_threshold

            # SUCCESS
            if dist_xy < 5.0 and at_final_alt:
                self.hover()
                print(f"\n  ARRIVED! Distance: {dist_xy:.1f}m")
                print(f"  Final position: ({x:.1f}, {y:.1f}, alt={-z:.1f}m)")
                return True

            # DESCEND
            elif over_target and not at_final_alt:
                phase = "DESCEND"
                vz = self.descent_speed
                correction_speed = min(1.0, dist_xy * 0.2)
                vx_world = correction_speed * math.cos(target_heading)
                vy_world = correction_speed * math.sin(target_heading)
                yaw_rate = 0.5 * yaw_error
                self.send_velocity(vx_world, vy_world, vz, yaw_rate)
                status = f"descending to {-target_z:.0f}m"

            # FINAL
            elif over_target and at_final_alt:
                phase = "FINAL"
                approach_speed = min(2.0, dist_xy * 0.3)
                vx_world = approach_speed * math.cos(target_heading)
                vy_world = approach_speed * math.sin(target_heading)
                yaw_rate = 0.5 * yaw_error
                self.send_velocity(vx_world, vy_world, 0, yaw_rate)
                status = f"approaching"

            # CRUISE
            else:
                phase = "CRUISE"
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
                status = f"v_n={vx_world:.1f} v_e={vy_world:.1f}"

            # Status
            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                print(f"  [{phase}] dist={dist_xy:.1f}m alt={-z:.1f}m yaw={yaw_deg:.0f}° {status}")

            time.sleep(0.05)

        self.hover()
        print(f"\n  Navigation timeout")
        return False

    def run(self, timeout: float = 120.0):
        """Run the mission."""
        self.running = True

        print("\n" + "=" * 60)
        print("Fly to Orange Ball - YOLO-World Edition")
        print("=" * 60)
        print(f"  Target: '{self.target_class}'")
        print(f"  Observation altitude: {-self.observation_altitude:.0f}m")
        print(f"  Approach altitude: {-self.approach_altitude:.0f}m")
        print("=" * 60)

        if not self.wait_for_data():
            print("ERROR: No detector data! Is yolo_world_detector.py running?")
            return

        x, y, z, yaw = self.get_pose()
        print(f"\nStarting pose: ({x:.1f}, {y:.1f}, {-z:.1f}m) yaw={math.degrees(yaw):.0f}°")

        start_time = time.time()

        # Execute phases
        if not self.phase1_ascend():
            return

        if not self.phase2_configure():
            return

        if not self.phase3_scan():
            return

        target = self.phase4_select()
        if not target:
            print("\nMission failed: No target found")
            return

        success = self.phase5_navigate(target)

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
        description="Fly to orange ball using YOLO-World detection",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Prerequisites:
  1. Start AirSim/Project AirSim with the drone scene
  2. Run the Zenoh bridge: python airsim_zenoh_bridge.py --connect tcp/localhost:7447
  3. Run YOLO detector: python yolo_world_detector.py --connect tcp/localhost:7447
  4. Run this mission: python fly_to_orange_ball_yolo.py --connect tcp/localhost:7447
        """
    )
    parser.add_argument("--connect", required=True, help="Zenoh endpoint")
    parser.add_argument("--robot-id", default="drone", help="Robot ID")
    parser.add_argument("--target", default="orange ball", help="Target class to find")
    parser.add_argument("--altitude", type=float, default=25.0, help="Observation altitude (m)")
    parser.add_argument("--approach", type=float, default=5.0, help="Approach altitude (m)")
    parser.add_argument("--timeout", type=float, default=120.0, help="Mission timeout (s)")

    args = parser.parse_args()

    mission = YoloOrangeBallMission(args.connect, args.robot_id)
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
