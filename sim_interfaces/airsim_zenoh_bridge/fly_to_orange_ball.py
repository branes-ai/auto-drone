#!/usr/bin/env python3
"""
Fly to Orange Ball Mission - Visual servoing to navigate towards an orange target.

This mission demonstrates perception-based navigation:
1. Subscribes to the RGB camera feed
2. Detects orange objects using color thresholding
3. Computes bearing to target center
4. Commands the drone to fly towards the orange ball

Usage:
    python fly_to_orange_ball.py --connect tcp/192.168.1.10:7447
"""

import argparse
import sys
import time
import threading
import math
import struct
from typing import Optional, Tuple
from dataclasses import dataclass

import numpy as np
import cv2
import zenoh

from data_types import (
    ImageData, Odometry, VelocityCommand,
    CommandPriority, CommandSource
)


@dataclass
class TargetDetection:
    """Detection result for the orange ball."""
    found: bool = False
    center_x: int = 0      # Pixel X coordinate of target center
    center_y: int = 0      # Pixel Y coordinate of target center
    area: int = 0          # Area in pixels (for distance estimation)
    image_width: int = 640
    image_height: int = 480

    @property
    def bearing_x(self) -> float:
        """Horizontal bearing: -1 (left) to +1 (right), 0 = centered."""
        if not self.found:
            return 0.0
        return (self.center_x - self.image_width / 2) / (self.image_width / 2)

    @property
    def bearing_y(self) -> float:
        """Vertical bearing: -1 (up) to +1 (down), 0 = centered."""
        if not self.found:
            return 0.0
        return (self.center_y - self.image_height / 2) / (self.image_height / 2)


class OrangeBallDetector:
    """Detects orange ball-shaped objects in RGB images using HSV color thresholding."""

    def __init__(self):
        # HSV range for orange color (tuned for typical orange)
        # Orange in HSV: Hue ~10-25, high Saturation, high Value
        self.hsv_lower = np.array([5, 100, 100])
        self.hsv_upper = np.array([25, 255, 255])
        self.min_area = 100  # Minimum blob area to consider
        self.min_circularity = 0.5  # Minimum circularity to be considered ball-like

    def _circularity(self, contour) -> float:
        """Calculate circularity: 4*pi*area/perimeter^2. Circle=1, rectangle<0.8."""
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return 0
        return 4 * math.pi * area / (perimeter * perimeter)

    def detect(self, image: np.ndarray) -> TargetDetection:
        """
        Detect orange ball in BGR image, preferring circular shapes.

        Args:
            image: BGR image from camera (numpy array)

        Returns:
            TargetDetection with results
        """
        if image is None or image.size == 0:
            return TargetDetection()

        height, width = image.shape[:2]
        result = TargetDetection(image_width=width, image_height=height)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold for orange color
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # Morphological operations to clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return result

        # Find the most ball-like orange blob (prefer circular over rectangular)
        best_contour = None
        best_score = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue

            circularity = self._circularity(contour)
            if circularity < self.min_circularity:
                continue  # Skip non-circular shapes (rectangles)

            # Score: prefer larger and more circular
            score = area * circularity
            if score > best_score:
                best_score = score
                best_contour = contour

        if best_contour is None:
            return result

        area = cv2.contourArea(best_contour)

        # Get centroid
        M = cv2.moments(best_contour)
        if M["m00"] == 0:
            return result

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        result.found = True
        result.center_x = cx
        result.center_y = cy
        result.area = int(area)

        return result


class FlyToOrangeBallMission:
    """Visual servoing mission to fly towards an orange ball."""

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

        # Detector
        self.detector = OrangeBallDetector()

        # Topics
        self.topic_rgb = f"robot/{robot_id}/sensor/camera/rgb"
        self.topic_odom = f"robot/{robot_id}/sensor/state/odom"
        self.topic_cmd_vel = f"robot/{robot_id}/cmd/velocity"

        # Control parameters
        self.forward_speed = 2.0      # m/s when target visible
        self.yaw_gain = 0.8           # Yaw rate proportional gain (reduced to prevent overshoot)
        self.vertical_gain = 0.3      # Vertical velocity gain
        self.approach_area = 50000    # Target area when "close enough"
        self.search_yaw_rate = 0.8    # Yaw rate when searching (rad/s)
        self.search_altitude = -10.0  # Target altitude for search (NED, negative = up)
        self.ascent_speed = 2.0       # m/s vertical speed when ascending
        self.centering_threshold = 0.3  # Only fly forward when bearing_x < this

        # Detection filtering (prevent oscillation from false positives)
        self.min_detection_area = 500     # Minimum blob area to consider valid
        self.detection_confirm_frames = 3  # Frames needed to confirm detection
        self.detection_counter = 0         # Current consecutive detection count
        self.tracking_active = False       # Whether we're in tracking mode

    def connect(self) -> bool:
        """Connect to Zenoh."""
        try:
            print(f"Connecting to Zenoh at {self.connect_endpoint}...")

            config = zenoh.Config()
            config.insert_json5("connect/endpoints", f'["{self.connect_endpoint}"]')

            self.session = zenoh.open(config)
            print("Connected to Zenoh successfully")

            # Subscribe to RGB camera
            self.session.declare_subscriber(self.topic_rgb, self._on_rgb)
            print(f"Subscribed to RGB: {self.topic_rgb}")

            # Subscribe to odometry
            self.session.declare_subscriber(self.topic_odom, self._on_odom)
            print(f"Subscribed to odometry: {self.topic_odom}")

            return True

        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def _on_rgb(self, sample: zenoh.Sample):
        """Handle RGB image updates."""
        try:
            img_data = ImageData.deserialize(bytes(sample.payload))
            img = img_data.to_numpy()
            with self.image_lock:
                self.latest_image = img
        except Exception as e:
            pass  # Silently ignore parse errors

    def _on_odom(self, sample: zenoh.Sample):
        """Handle odometry updates."""
        try:
            odom = Odometry.deserialize(bytes(sample.payload))
            with self.odom_lock:
                self.latest_odom = odom
        except Exception:
            pass

    def get_image(self) -> Optional[np.ndarray]:
        """Get latest camera image."""
        with self.image_lock:
            return self.latest_image.copy() if self.latest_image is not None else None

    def get_position(self) -> Tuple[float, float, float]:
        """Get current position (x, y, z)."""
        with self.odom_lock:
            if self.latest_odom:
                return (self.latest_odom.x, self.latest_odom.y, self.latest_odom.z)
        return (0, 0, 0)

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """Send velocity command."""
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

    def run(self, timeout: float = 120.0):
        """Run the mission."""
        self.running = True

        print("\n" + "=" * 60)
        print("Fly to Orange Ball Mission")
        print("=" * 60)
        print("\nUsing front camera visual servoing to navigate to orange target")
        print(f"Search altitude: {-self.search_altitude:.0f}m")
        print(f"Timeout: {timeout}s")
        print("=" * 60 + "\n")

        # Wait for camera feed
        print("Waiting for camera feed...")
        for _ in range(50):
            if self.latest_image is not None:
                break
            time.sleep(0.1)

        if self.latest_image is None:
            print("ERROR: No camera feed received!")
            return

        pos = self.get_position()
        print(f"Starting position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")

        # Phase 1: Ascend to search altitude
        print(f"\nPhase 1: Ascending to search altitude ({-self.search_altitude:.0f}m)...")
        while self.running and pos[2] > self.search_altitude + 0.5:
            self.send_velocity(0, 0, -self.ascent_speed, 0)  # negative vz = ascend
            time.sleep(0.1)
            pos = self.get_position()
            print(f"  Altitude: {-pos[2]:.1f}m", end='\r')

        self.hover()
        pos = self.get_position()
        print(f"\nReached search altitude: {-pos[2]:.1f}m")

        # Phase 2: Search for target
        print("\nPhase 2: Searching for orange ball...")
        print("Press Ctrl+C to stop.\n")

        # Control loop
        rate = 20.0  # Hz
        period = 1.0 / rate
        start_time = time.time()
        last_status_time = start_time
        frames_processed = 0
        target_reached = False

        while self.running and (time.time() - start_time) < timeout:
            loop_start = time.time()

            # Get latest image
            image = self.get_image()
            if image is None:
                time.sleep(period)
                continue

            # Detect orange ball
            detection = self.detector.detect(image)
            frames_processed += 1

            # Filter detections - require minimum area and consecutive frames
            valid_detection = detection.found and detection.area >= self.min_detection_area

            # Update detection counter with hysteresis
            if valid_detection:
                self.detection_counter = min(self.detection_counter + 1, self.detection_confirm_frames + 5)
                if self.detection_counter >= self.detection_confirm_frames:
                    self.tracking_active = True
            else:
                self.detection_counter = max(self.detection_counter - 1, 0)
                # Require several misses before going back to search (hysteresis)
                if self.detection_counter == 0:
                    self.tracking_active = False

            # Compute velocity command based on tracking state
            if self.tracking_active and valid_detection:
                # Visual servoing: fly towards target
                # Yaw to center target horizontally
                # Positive bearing_x = target on right = need positive yaw (turn right)
                yaw_rate = self.yaw_gain * detection.bearing_x

                # Don't adjust altitude - maintain current altitude
                # (Vertical tracking causes descent towards ground obstacles)
                vz = 0.0

                # Only fly forward when target is reasonably centered
                if abs(detection.bearing_x) > self.centering_threshold:
                    # Target not centered - just rotate to center it
                    vx = 0.0
                    mode = "CENTERING"
                elif detection.area > self.approach_area:
                    # Close enough - stop
                    vx = 0.0
                    target_reached = True
                    mode = "ARRIVED"
                else:
                    # Target centered - fly forward
                    # Scale forward speed based on target size (closer = bigger = slower)
                    scale = max(0.2, 1.0 - detection.area / self.approach_area)
                    vx = self.forward_speed * scale
                    mode = "APPROACHING"

                self.send_velocity(vx, 0, vz, yaw_rate)

                # Status update
                if time.time() - last_status_time > 1.0:
                    last_status_time = time.time()
                    pos = self.get_position()
                    print(f"  {mode}: bearing=({detection.bearing_x:.2f}, {detection.bearing_y:.2f}) "
                          f"area={detection.area} pos=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")

                if target_reached:
                    print("\n*** TARGET REACHED! ***")
                    self.hover()
                    break

            else:
                # No confirmed target - rotate in place to scan horizon (front camera)
                self.send_velocity(0, 0, 0, self.search_yaw_rate)

                if time.time() - last_status_time > 2.0:
                    last_status_time = time.time()
                    pos = self.get_position()
                    status = f"(detecting: {self.detection_counter}/{self.detection_confirm_frames})" if valid_detection else ""
                    print(f"  Rotating search {status}... "
                          f"pos=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")

            # Maintain loop rate
            elapsed = time.time() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)

        # Final status
        self.hover()
        pos = self.get_position()
        print(f"\nMission ended. Final position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
        print(f"Frames processed: {frames_processed}")

        if target_reached:
            print("RESULT: SUCCESS - Reached orange ball!")
        elif not self.running:
            print("RESULT: INTERRUPTED")
        else:
            print("RESULT: TIMEOUT - Did not reach target")

    def stop(self):
        """Stop the mission."""
        self.running = False

    def disconnect(self):
        """Close Zenoh session."""
        if self.session:
            self.session.close()
            self.session = None


def main():
    parser = argparse.ArgumentParser(description="Fly to orange ball using visual servoing")
    parser.add_argument("--connect", required=True, help="Zenoh endpoint (e.g., tcp/192.168.1.10:7447)")
    parser.add_argument("--robot-id", default="drone", help="Robot ID (default: drone)")
    parser.add_argument("--timeout", type=float, default=120.0, help="Mission timeout in seconds")
    parser.add_argument("--speed", type=float, default=1.0, help="Forward speed m/s (default: 1.0)")
    parser.add_argument("--altitude", type=float, default=10.0, help="Search altitude in meters (default: 10)")

    args = parser.parse_args()

    mission = FlyToOrangeBallMission(args.connect, args.robot_id)
    mission.forward_speed = args.speed
    mission.search_altitude = -args.altitude  # Convert to NED (negative = up)

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
