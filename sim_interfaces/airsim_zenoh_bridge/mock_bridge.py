#!/usr/bin/env python3
"""
Mock AirSim Zenoh Bridge for local testing.

This simulates the real airsim_zenoh_bridge.py without requiring Project AirSim.
Use this to test Zenoh connectivity and data serialization on Linux.
"""

import argparse
import signal
import sys
import time
import threading
import math
import random
from dataclasses import dataclass
from typing import Optional

import numpy as np
import zenoh

from data_types import (
    ImageData, ImageEncoding, Odometry, VelocityCommand,
    CommandPriority, CommandSource
)

NOISE_LEVELS = {
    "0": 0.0,
    "low": 0.05,
    "medium": 0.10,
    "high": 0.20,
    "very_high": 0.40,
}


@dataclass
class MockDroneState:
    """Simulated drone state."""
    x: float = 0.0
    y: float = 0.0
    z: float = -5.0  # NED: negative = above ground
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Velocities (for command response)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


class MockAirSimBridge:
    """Mock bridge that simulates AirSim sensor data."""

    def __init__(
        self,
        zenoh_listen: Optional[str] = None,
        robot_id: str = "drone",
        start_x: float = 0.0,
        start_y: float = 0.0,
        start_z: float = -5.0,
        start_yaw_rad: float = 0.0,
        actuator_noise_level: float = 0.0,
        seed: Optional[int] = None,
    ):
        self.zenoh_listen = zenoh_listen
        self.robot_id = robot_id
        self.running = False
        self.session: Optional[zenoh.Session] = None

        self.start_pose = (start_x, start_y, start_z, start_yaw_rad)
        self.actuator_noise_level = actuator_noise_level
        self.rng = random.Random(seed)

        # Simulated drone state
        self.state = MockDroneState(x=start_x, y=start_y, z=start_z, yaw=start_yaw_rad)
        self.state_lock = threading.Lock()

        # Command state
        self.last_command_time = 0.0
        self.command_timeout = 0.5

        # Statistics
        self.stats = {
            'rgb_frames': 0,
            'depth_frames': 0,
            'odom_messages': 0,
            'commands_received': 0,
        }

        # Topics
        self.topic_rgb = f"robot/{robot_id}/sensor/camera/rgb"
        self.topic_depth = f"robot/{robot_id}/sensor/camera/depth"
        self.topic_odom = f"robot/{robot_id}/sensor/state/odom"
        self.topic_cmd_vel = f"robot/{robot_id}/cmd/velocity"

    def connect(self) -> bool:
        """Initialize Zenoh session."""
        try:
            print("Initializing Zenoh session...")

            config = zenoh.Config()

            if self.zenoh_listen:
                config.insert_json5("listen/endpoints", f'["{self.zenoh_listen}"]')
                print(f"  Listening on: {self.zenoh_listen}")
            else:
                print("  Using local scouting (default)")

            self.session = zenoh.open(config)
            print("Zenoh session opened successfully")

            # Create publishers
            self.pub_rgb = self.session.declare_publisher(self.topic_rgb)
            self.pub_depth = self.session.declare_publisher(self.topic_depth)
            self.pub_odom = self.session.declare_publisher(self.topic_odom)

            print(f"  Publishing RGB to: {self.topic_rgb}")
            print(f"  Publishing Depth to: {self.topic_depth}")
            print(f"  Publishing Odom to: {self.topic_odom}")

            # Subscribe to velocity commands
            self.sub_cmd = self.session.declare_subscriber(
                self.topic_cmd_vel,
                self._on_velocity_command
            )
            print(f"  Subscribed to: {self.topic_cmd_vel}")

            return True

        except Exception as e:
            print(f"Failed to initialize Zenoh: {e}")
            return False

    def _apply_relative_noise(self, value: float) -> float:
        if self.actuator_noise_level <= 0.0 or value == 0.0:
            return value
        return value * (1.0 + self.rng.uniform(-self.actuator_noise_level, self.actuator_noise_level))

    def _on_velocity_command(self, sample: zenoh.Sample):
        """Handle incoming velocity commands."""
        try:
            cmd = VelocityCommand.deserialize(bytes(sample.payload))

            with self.state_lock:
                self.state.vx = self._apply_relative_noise(cmd.vx)
                self.state.vy = cmd.vy
                self.state.vz = self._apply_relative_noise(cmd.vz)
                self.state.yaw_rate = self._apply_relative_noise(cmd.yaw_rate)
                self.last_command_time = time.time()

            self.stats['commands_received'] += 1

        except Exception as e:
            print(f"Error parsing command: {e}")

    def _update_state(self, dt: float):
        """Update simulated drone state based on velocity commands."""
        with self.state_lock:
            # Check command timeout
            if time.time() - self.last_command_time > self.command_timeout:
                self.state.vx = 0.0
                self.state.vy = 0.0
                self.state.vz = 0.0
                self.state.yaw_rate = 0.0

            # Simple kinematic update (body frame to world frame)
            cos_yaw = math.cos(self.state.yaw)
            sin_yaw = math.sin(self.state.yaw)

            # Transform body velocities to world frame
            world_vx = self.state.vx * cos_yaw - self.state.vy * sin_yaw
            world_vy = self.state.vx * sin_yaw + self.state.vy * cos_yaw

            self.state.x += world_vx * dt
            self.state.y += world_vy * dt
            self.state.z += self.state.vz * dt
            self.state.yaw += self.state.yaw_rate * dt

            # Wrap yaw to [-pi, pi]
            while self.state.yaw > math.pi:
                self.state.yaw -= 2 * math.pi
            while self.state.yaw < -math.pi:
                self.state.yaw += 2 * math.pi

    def _publish_odometry(self):
        """Publish simulated odometry."""
        with self.state_lock:
            odom = Odometry(
                x=self.state.x,
                y=self.state.y,
                z=self.state.z,
                roll=self.state.roll,
                pitch=self.state.pitch,
                yaw=self.state.yaw,
                timestamp_us=int(time.time() * 1_000_000)
            )

        self.pub_odom.put(odom.serialize())
        self.stats['odom_messages'] += 1

    def _publish_rgb(self):
        """Publish simulated RGB image (gradient pattern)."""
        width, height = 640, 480
        t = time.time()

        img = np.zeros((height, width, 3), dtype=np.uint8)

        for x in range(width):
            r = int(128 + 127 * math.sin(x / 100.0 + t))
            g = int(128 + 127 * math.sin(x / 100.0 + t + 2.094))
            b = int(128 + 127 * math.sin(x / 100.0 + t + 4.188))
            img[:, x] = [b, g, r]

        cx = int(width / 2 + width / 4 * math.sin(t))
        cy = int(height / 2 + height / 4 * math.cos(t * 0.7))
        try:
            import cv2
            cv2.circle(img, (cx, cy), 30, (0, 255, 0), -1)
        except ImportError:
            for dy in range(-30, 31):
                for dx in range(-30, 31):
                    if dx * dx + dy * dy <= 900:
                        py, px = cy + dy, cx + dx
                        if 0 <= py < height and 0 <= px < width:
                            img[py, px] = [0, 255, 0]

        image_data = ImageData.from_numpy(img, ImageEncoding.BGR8)
        self.pub_rgb.put(image_data.serialize())
        self.stats['rgb_frames'] += 1

    def _publish_depth(self):
        """Publish simulated depth image."""
        width, height = 640, 480
        y_coords, x_coords = np.ogrid[:height, :width]
        cx, cy = width // 2, height // 2
        dist = np.sqrt((x_coords - cx) ** 2 + (y_coords - cy) ** 2)
        max_dist = np.sqrt(cx ** 2 + cy ** 2)
        depth = (dist / max_dist * 200 + 20).astype(np.uint8)

        image_data = ImageData.from_numpy(depth, ImageEncoding.GRAY8)
        self.pub_depth.put(image_data.serialize())
        self.stats['depth_frames'] += 1

    def _sensor_loop(self, publish_func, rate: float):
        """Generic sensor publishing loop."""
        period = 1.0 / rate
        while self.running:
            start = time.time()
            publish_func()
            elapsed = time.time() - start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _state_update_loop(self):
        """State update loop (100 Hz)."""
        rate = 100.0
        period = 1.0 / rate
        while self.running:
            start = time.time()
            self._update_state(period)
            elapsed = time.time() - start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def run(self):
        """Main run loop."""
        if not self.connect():
            return False

        self.running = True
        print("\n" + "=" * 60)
        print("Mock AirSim Bridge running")
        print("=" * 60)
        print(
            f"Simulating drone at initial position: "
            f"({self.start_pose[0]}, {self.start_pose[1]}, {self.start_pose[2]}) yaw={math.degrees(self.start_pose[3]):.1f}deg"
        )
        print(f"Actuator noise level: {self.actuator_noise_level:.2f}")
        print("Press Ctrl+C to stop.\n")

        threads = [
            threading.Thread(target=self._state_update_loop, daemon=True),
            threading.Thread(target=self._sensor_loop, args=(self._publish_odometry, 100.0), daemon=True),
            threading.Thread(target=self._sensor_loop, args=(self._publish_rgb, 30.0), daemon=True),
            threading.Thread(target=self._sensor_loop, args=(self._publish_depth, 30.0), daemon=True),
        ]

        for t in threads:
            t.start()

        try:
            while self.running:
                time.sleep(5.0)
                self._print_stats()
        except KeyboardInterrupt:
            pass

        self.shutdown()
        return True

    def _print_stats(self):
        """Print bridge statistics."""
        with self.state_lock:
            pos = f"({self.state.x:.2f}, {self.state.y:.2f}, {self.state.z:.2f})"

        print(f"Stats: RGB={self.stats['rgb_frames']} | "
              f"Depth={self.stats['depth_frames']} | "
              f"Odom={self.stats['odom_messages']} | "
              f"CmdRx={self.stats['commands_received']} | "
              f"Pos={pos}")

    def shutdown(self):
        """Clean shutdown."""
        print("\nShutting down...")
        self.running = False

        if self.session:
            self.session.close()

        print("Shutdown complete")


def main():
    parser = argparse.ArgumentParser(
        description="Mock AirSim Zenoh Bridge for local testing",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument('--zenoh-listen', type=str, default=None,
                       help='Zenoh endpoint to listen on (e.g., tcp/0.0.0.0:7447)')
    parser.add_argument('--robot-id', type=str, default='drone',
                       help='Robot identifier for Zenoh topics')
    parser.add_argument('--start-x', type=float, default=0.0,
                       help='Initial X position (meters)')
    parser.add_argument('--start-y', type=float, default=0.0,
                       help='Initial Y position (meters)')
    parser.add_argument('--start-z', type=float, default=-5.0,
                       help='Initial Z position in NED (negative is above ground)')
    parser.add_argument('--start-yaw-deg', type=float, default=0.0,
                       help='Initial yaw in degrees')
    parser.add_argument('--actuator-noise-level', choices=list(NOISE_LEVELS.keys()), default='0',
                       help='Named actuator noise level applied to vx/vz/yaw_rate')
    parser.add_argument('--seed', type=int, default=None,
                       help='Optional random seed for reproducible actuator noise')

    args = parser.parse_args()

    bridge = MockAirSimBridge(
        zenoh_listen=args.zenoh_listen,
        robot_id=args.robot_id,
        start_x=args.start_x,
        start_y=args.start_y,
        start_z=args.start_z,
        start_yaw_rad=math.radians(args.start_yaw_deg),
        actuator_noise_level=NOISE_LEVELS[args.actuator_noise_level],
        seed=args.seed,
    )

    def signal_handler(sig, frame):
        bridge.running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    success = bridge.run()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
