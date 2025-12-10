#!/usr/bin/env python3
"""
Project AirSim to Zenoh Bridge

This bridge runs on the same machine as Project AirSim (Windows) and:
- Connects to Project AirSim via localhost Python API
- Publishes sensor data (RGB, depth, odometry) to Zenoh
- Subscribes to velocity commands from Zenoh and forwards to AirSim

This enables remote autonomy stacks to control the simulated drone over the network.
"""

import argparse
import signal
import sys
import time
import threading
from dataclasses import dataclass
from typing import Optional
import math

import numpy as np
import zenoh

# Project AirSim imports
from projectairsim import ProjectAirSimClient, Drone, World

from data_types import (
    ImageData, ImageEncoding, Odometry, VelocityCommand,
    CommandPriority, CommandSource
)


@dataclass
class BridgeConfig:
    """Configuration for the AirSim-Zenoh bridge."""
    # Zenoh configuration
    zenoh_connect: Optional[str] = None  # e.g., "tcp/192.168.1.20:7447"
    zenoh_listen: Optional[str] = None   # e.g., "tcp/0.0.0.0:7447"

    # Topic configuration
    robot_id: str = "drone"
    camera_name: str = "front_center"

    # Sensor rates (Hz)
    rgb_rate: float = 30.0
    depth_rate: float = 30.0
    odom_rate: float = 100.0

    # Image configuration
    image_width: int = 640
    image_height: int = 480

    # Control configuration
    command_timeout: float = 0.5  # seconds before hover if no commands received

    # Flight configuration
    auto_takeoff: bool = False
    takeoff_altitude: float = 10.0


class AirSimZenohBridge:
    """Bridge between Project AirSim and Zenoh network."""

    def __init__(self, config: BridgeConfig):
        self.config = config
        self.running = False
        self.drone: Optional[Drone] = None
        self.world: Optional[World] = None
        self.zenoh_session: Optional[zenoh.Session] = None

        # Command state
        self.last_command_time = 0.0
        self.current_command = VelocityCommand.hover()
        self.command_lock = threading.Lock()

        # Statistics
        self.stats = {
            'rgb_frames': 0,
            'depth_frames': 0,
            'odom_messages': 0,
            'commands_received': 0,
            'commands_applied': 0,
        }

        # Topic key expressions
        self.topic_rgb = f"robot/{config.robot_id}/sensor/camera/rgb"
        self.topic_depth = f"robot/{config.robot_id}/sensor/camera/depth"
        self.topic_odom = f"robot/{config.robot_id}/sensor/state/odom"
        self.topic_cmd_vel = f"robot/{config.robot_id}/cmd/velocity"

    def connect_airsim(self) -> bool:
        """Connect to Project AirSim."""
        try:
            print("Connecting to Project AirSim...")
            client = ProjectAirSimClient()
            self.world = World(client)
            self.drone = Drone(client, "Drone")

            # Enable API control
            self.drone.enable_api_control()
            print("Connected to Project AirSim successfully")

            if self.config.auto_takeoff:
                print(f"Taking off to altitude {self.config.takeoff_altitude}m...")
                self.drone.arm()
                self.drone.takeoff(self.config.takeoff_altitude)
                print("Takeoff complete")

            return True

        except Exception as e:
            print(f"Failed to connect to Project AirSim: {e}")
            return False

    def connect_zenoh(self) -> bool:
        """Initialize Zenoh session."""
        try:
            print("Initializing Zenoh session...")

            # Build Zenoh configuration
            zenoh_config = zenoh.Config()

            if self.config.zenoh_connect:
                zenoh_config.insert_json5("connect/endpoints", f'["{self.config.zenoh_connect}"]')
                print(f"  Zenoh connect: {self.config.zenoh_connect}")

            if self.config.zenoh_listen:
                zenoh_config.insert_json5("listen/endpoints", f'["{self.config.zenoh_listen}"]')
                print(f"  Zenoh listen: {self.config.zenoh_listen}")

            self.zenoh_session = zenoh.open(zenoh_config)
            print("Zenoh session opened successfully")

            # Create publishers
            self.pub_rgb = self.zenoh_session.declare_publisher(self.topic_rgb)
            self.pub_depth = self.zenoh_session.declare_publisher(self.topic_depth)
            self.pub_odom = self.zenoh_session.declare_publisher(self.topic_odom)
            print(f"  Publishing RGB to: {self.topic_rgb}")
            print(f"  Publishing Depth to: {self.topic_depth}")
            print(f"  Publishing Odometry to: {self.topic_odom}")

            # Create subscriber for velocity commands
            self.sub_cmd_vel = self.zenoh_session.declare_subscriber(
                self.topic_cmd_vel,
                self._on_velocity_command
            )
            print(f"  Subscribed to commands: {self.topic_cmd_vel}")

            return True

        except Exception as e:
            print(f"Failed to initialize Zenoh: {e}")
            return False

    def _on_velocity_command(self, sample: zenoh.Sample):
        """Handle incoming velocity commands from Zenoh."""
        try:
            cmd = VelocityCommand.deserialize(bytes(sample.payload))

            with self.command_lock:
                self.current_command = cmd
                self.last_command_time = time.time()
                self.stats['commands_received'] += 1

        except Exception as e:
            print(f"Error parsing velocity command: {e}")

    def _get_timestamp_us(self) -> int:
        """Get current timestamp in microseconds."""
        return int(time.time() * 1_000_000)

    def _quaternion_to_euler(self, w: float, x: float, y: float, z: float) -> tuple:
        """Convert quaternion to Euler angles (roll, pitch, yaw) in radians."""
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _publish_odometry(self):
        """Read and publish odometry from AirSim."""
        try:
            # Get pose from AirSim
            pose = self.drone.get_pose()

            # Extract position
            x = pose.position.x_val
            y = pose.position.y_val
            z = pose.position.z_val

            # Convert quaternion to Euler
            q = pose.orientation
            roll, pitch, yaw = self._quaternion_to_euler(q.w_val, q.x_val, q.y_val, q.z_val)

            odom = Odometry(
                x=x, y=y, z=z,
                roll=roll, pitch=pitch, yaw=yaw,
                timestamp_us=self._get_timestamp_us()
            )

            self.pub_odom.put(odom.serialize())
            self.stats['odom_messages'] += 1

        except Exception as e:
            print(f"Error publishing odometry: {e}")

    def _publish_rgb(self):
        """Capture and publish RGB image from AirSim."""
        try:
            # Request RGB image from AirSim
            responses = self.drone.get_images([{
                'camera_name': self.config.camera_name,
                'image_type': 0,  # Scene (RGB)
                'pixels_as_float': False,
                'compress': False,
            }])

            if responses and len(responses) > 0:
                response = responses[0]

                # Convert to numpy array
                img_1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                img = img_1d.reshape(response.height, response.width, 3)

                # Create ImageData (AirSim returns BGR by default)
                image_data = ImageData.from_numpy(img, ImageEncoding.BGR8)

                self.pub_rgb.put(image_data.serialize())
                self.stats['rgb_frames'] += 1

        except Exception as e:
            print(f"Error publishing RGB image: {e}")

    def _publish_depth(self):
        """Capture and publish depth image from AirSim."""
        try:
            # Request depth image from AirSim
            responses = self.drone.get_images([{
                'camera_name': self.config.camera_name,
                'image_type': 2,  # DepthPerspective
                'pixels_as_float': True,
                'compress': False,
            }])

            if responses and len(responses) > 0:
                response = responses[0]

                # Convert float depth to normalized uint8 for transmission
                depth_float = np.array(response.image_data_float, dtype=np.float32)
                depth_float = depth_float.reshape(response.height, response.width)

                # Normalize depth to 0-255 (assuming max depth of 100m)
                max_depth = 100.0
                depth_normalized = np.clip(depth_float / max_depth * 255, 0, 255).astype(np.uint8)

                image_data = ImageData.from_numpy(depth_normalized, ImageEncoding.GRAY8)

                self.pub_depth.put(image_data.serialize())
                self.stats['depth_frames'] += 1

        except Exception as e:
            print(f"Error publishing depth image: {e}")

    def _apply_command(self):
        """Apply current velocity command to AirSim."""
        try:
            with self.command_lock:
                # Check for command timeout
                if time.time() - self.last_command_time > self.config.command_timeout:
                    cmd = VelocityCommand.hover()
                else:
                    cmd = self.current_command

            # Apply velocity command to AirSim
            # Note: AirSim uses NED frame, body frame velocities
            self.drone.move_by_velocity(
                vx=cmd.vx,
                vy=cmd.vy,
                vz=cmd.vz,
                duration=0.1,  # Short duration, will be refreshed
                yaw_mode={'is_rate': True, 'yaw_or_rate': cmd.yaw_rate}
            )
            self.stats['commands_applied'] += 1

        except Exception as e:
            print(f"Error applying command: {e}")

    def _sensor_loop(self, publish_func, rate: float, name: str):
        """Generic sensor publishing loop."""
        period = 1.0 / rate
        while self.running:
            start = time.time()
            publish_func()
            elapsed = time.time() - start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _command_loop(self):
        """Command application loop."""
        rate = 50.0  # 50 Hz command rate
        period = 1.0 / rate
        while self.running:
            start = time.time()
            self._apply_command()
            elapsed = time.time() - start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def run(self):
        """Main run loop."""
        if not self.connect_airsim():
            return False

        if not self.connect_zenoh():
            return False

        self.running = True
        print("\nBridge running. Press Ctrl+C to stop.\n")

        # Start sensor threads
        threads = []

        odom_thread = threading.Thread(
            target=self._sensor_loop,
            args=(self._publish_odometry, self.config.odom_rate, "odom"),
            daemon=True
        )
        threads.append(odom_thread)

        rgb_thread = threading.Thread(
            target=self._sensor_loop,
            args=(self._publish_rgb, self.config.rgb_rate, "rgb"),
            daemon=True
        )
        threads.append(rgb_thread)

        depth_thread = threading.Thread(
            target=self._sensor_loop,
            args=(self._publish_depth, self.config.depth_rate, "depth"),
            daemon=True
        )
        threads.append(depth_thread)

        cmd_thread = threading.Thread(
            target=self._command_loop,
            daemon=True
        )
        threads.append(cmd_thread)

        for t in threads:
            t.start()

        # Print statistics periodically
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
        print(f"Stats: RGB={self.stats['rgb_frames']} | "
              f"Depth={self.stats['depth_frames']} | "
              f"Odom={self.stats['odom_messages']} | "
              f"CmdRx={self.stats['commands_received']} | "
              f"CmdTx={self.stats['commands_applied']}")

    def shutdown(self):
        """Clean shutdown."""
        print("\nShutting down...")
        self.running = False

        # Hover and disarm
        if self.drone:
            try:
                self.drone.hover()
                time.sleep(0.5)
                self.drone.land()
                self.drone.disarm()
                self.drone.disable_api_control()
            except Exception as e:
                print(f"Error during shutdown: {e}")

        # Close Zenoh session
        if self.zenoh_session:
            self.zenoh_session.close()

        print("Shutdown complete")


def main():
    parser = argparse.ArgumentParser(
        description="Project AirSim to Zenoh Bridge",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    # Zenoh configuration
    parser.add_argument('--zenoh-connect', type=str, default=None,
                       help='Zenoh endpoint to connect to (e.g., tcp/192.168.1.20:7447)')
    parser.add_argument('--zenoh-listen', type=str, default=None,
                       help='Zenoh endpoint to listen on (e.g., tcp/0.0.0.0:7447)')

    # Robot configuration
    parser.add_argument('--robot-id', type=str, default='drone',
                       help='Robot identifier for Zenoh topics')
    parser.add_argument('--camera', type=str, default='front_center',
                       help='Camera name in AirSim')

    # Rate configuration
    parser.add_argument('--rgb-rate', type=float, default=30.0,
                       help='RGB image publish rate (Hz)')
    parser.add_argument('--depth-rate', type=float, default=30.0,
                       help='Depth image publish rate (Hz)')
    parser.add_argument('--odom-rate', type=float, default=100.0,
                       help='Odometry publish rate (Hz)')

    # Image configuration
    parser.add_argument('--width', type=int, default=640,
                       help='Image width')
    parser.add_argument('--height', type=int, default=480,
                       help='Image height')

    # Flight configuration
    parser.add_argument('--auto-takeoff', action='store_true',
                       help='Automatically takeoff on start')
    parser.add_argument('--altitude', type=float, default=10.0,
                       help='Takeoff altitude (meters)')

    args = parser.parse_args()

    config = BridgeConfig(
        zenoh_connect=args.zenoh_connect,
        zenoh_listen=args.zenoh_listen,
        robot_id=args.robot_id,
        camera_name=args.camera,
        rgb_rate=args.rgb_rate,
        depth_rate=args.depth_rate,
        odom_rate=args.odom_rate,
        image_width=args.width,
        image_height=args.height,
        auto_takeoff=args.auto_takeoff,
        takeoff_altitude=args.altitude,
    )

    bridge = AirSimZenohBridge(config)

    # Handle signals
    def signal_handler(sig, frame):
        bridge.running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    success = bridge.run()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
