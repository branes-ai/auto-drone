#!/usr/bin/env python
"""
Project AirSim to Zenoh Bridge

This bridge runs on the same machine as Project AirSim (Windows) and:
- Connects to Project AirSim via localhost Python API
- Publishes sensor data (RGB, depth, odometry) to Zenoh
- Subscribes to velocity commands from Zenoh and forwards to AirSim

This enables remote autonomy stacks to control the simulated drone over the network.
"""

import argparse
import asyncio
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

    # AirSim configuration
    drone_name: str = "Drone1"
    scene_file: str = "scene_basic_drone.jsonc"  # Scene file to load

    # Sensor rates (Hz)
    rgb_rate: float = 30.0
    depth_rate: float = 30.0
    odom_rate: float = 100.0

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
        self.client: Optional[ProjectAirSimClient] = None
        self.drone: Optional[Drone] = None
        self.world: Optional[World] = None
        self.zenoh_session: Optional[zenoh.Session] = None

        # Command state
        self.last_command_time = 0.0
        self.current_command = VelocityCommand.hover()
        self.command_lock = threading.Lock()

        # Async event loop for AirSim commands
        self.loop: Optional[asyncio.AbstractEventLoop] = None

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

        # Sensor data from subscriptions
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_odom = None
        self.rgb_lock = threading.Lock()
        self.depth_lock = threading.Lock()
        self.odom_lock = threading.Lock()

    async def connect_airsim(self) -> bool:
        """Connect to Project AirSim."""
        try:
            print("Connecting to Project AirSim...")

            self.client = ProjectAirSimClient()
            self.client.connect()
            print("  Client connected")

            # Load scene - this is required by Project AirSim
            print(f"  Loading scene: {self.config.scene_file}")
            self.world = World(self.client, self.config.scene_file, delay_after_load_sec=2)
            print("  Scene loaded")

            # Create drone
            print(f"  Getting drone: {self.config.drone_name}")
            self.drone = Drone(self.client, self.world, self.config.drone_name)

            # Enable API control
            self.drone.enable_api_control()
            print("Connected to Project AirSim successfully")

            if self.config.auto_takeoff:
                print(f"Taking off to altitude {self.config.takeoff_altitude}m...")
                self.drone.arm()
                takeoff_task = await self.drone.takeoff_async()
                await takeoff_task
                print("Takeoff complete")

            # Subscribe to sensor feeds using Project AirSim's subscription mechanism
            self._setup_sensor_subscriptions()

            return True

        except Exception as e:
            print(f"Failed to connect to Project AirSim: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _setup_sensor_subscriptions(self):
        """Set up subscriptions to sensor feeds from Project AirSim."""

        # Callback for RGB camera
        def on_rgb_image(topic, image_data):
            try:
                # Project AirSim sends image as dict:
                # {'time_stamp': ..., 'height': ..., 'width': ..., 'encoding': 'BGR', 'data': bytes}
                if isinstance(image_data, dict) and 'data' in image_data:
                    height = image_data.get('height', 0)
                    width = image_data.get('width', 0)
                    encoding = image_data.get('encoding', 'BGR')
                    raw_data = image_data.get('data', b'')

                    if height > 0 and width > 0 and raw_data:
                        # Convert to numpy array
                        img = np.frombuffer(raw_data, dtype=np.uint8)
                        channels = 3 if encoding in ('BGR', 'RGB') else 1
                        img = img.reshape(height, width, channels)

                        with self.rgb_lock:
                            self.latest_rgb = img
            except Exception as e:
                if self.stats['rgb_frames'] == 0:
                    print(f"  Error parsing RGB: {e}")

        # Callback for depth camera
        def on_depth_image(topic, image_data):
            try:
                # Same format as RGB but different pixel format
                if isinstance(image_data, dict) and 'data' in image_data:
                    height = image_data.get('height', 0)
                    width = image_data.get('width', 0)
                    raw_data = image_data.get('data', b'')

                    if height > 0 and width > 0 and raw_data:
                        pixel_count = height * width
                        data_size = len(raw_data)

                        # Determine depth format based on data size
                        if data_size == pixel_count * 4:
                            # float32 (4 bytes per pixel)
                            img = np.frombuffer(raw_data, dtype=np.float32).reshape(height, width)
                        elif data_size == pixel_count * 2:
                            # uint16 (2 bytes per pixel) - convert to float for normalization
                            img = np.frombuffer(raw_data, dtype=np.uint16).reshape(height, width).astype(np.float32)
                        elif data_size == pixel_count:
                            # uint8 (1 byte per pixel)
                            img = np.frombuffer(raw_data, dtype=np.uint8).reshape(height, width).astype(np.float32)
                        else:
                            # Unknown format - skip
                            return

                        with self.depth_lock:
                            self.latest_depth = img
            except Exception as e:
                if self.stats['depth_frames'] == 0:
                    print(f"  Error parsing depth: {e} (size={len(raw_data)}, expected={height}x{width})")

        # Callback for pose/odometry (from robot_info or IMU)
        def on_pose(topic, pose_data):
            try:
                # Project AirSim sends pose as a dict:
                # {'time_stamp': ..., 'position': {'x': ..., 'y': ..., 'z': ...},
                #  'orientation': {'w': ..., 'x': ..., 'y': ..., 'z': ...}}
                if isinstance(pose_data, dict):
                    pos = pose_data.get('position', {})
                    ori = pose_data.get('orientation', {})

                    x = pos.get('x', 0)
                    y = pos.get('y', 0)
                    z = pos.get('z', 0)

                    # Convert quaternion to euler
                    w = ori.get('w', 1)
                    qx = ori.get('x', 0)
                    qy = ori.get('y', 0)
                    qz = ori.get('z', 0)
                    roll, pitch, yaw = self._quaternion_to_euler(w, qx, qy, qz)

                    odom = Odometry(
                        x=x, y=y, z=z,
                        roll=roll, pitch=pitch, yaw=yaw,
                        timestamp_us=self._get_timestamp_us()
                    )
                    with self.odom_lock:
                        self.latest_odom = odom

            except Exception as e:
                if self.stats['odom_messages'] == 0:
                    print(f"  Error parsing pose: {e} - data type: {type(pose_data)}")

        try:
            # Project AirSim uses drone.sensors and drone.robot_info for topics
            print(f"  Setting up sensor subscriptions...")

            # Subscribe to cameras
            if hasattr(self.drone, 'sensors'):
                print(f"  Available sensors: {list(self.drone.sensors.keys())}")

                # Try DownCamera first (used in hello_drone.py example)
                if "DownCamera" in self.drone.sensors:
                    cam = self.drone.sensors["DownCamera"]
                    if "scene_camera" in cam:
                        self.client.subscribe(cam["scene_camera"], on_rgb_image)
                        print(f"    Subscribed to DownCamera RGB")
                    if "depth_camera" in cam:
                        self.client.subscribe(cam["depth_camera"], on_depth_image)
                        print(f"    Subscribed to DownCamera Depth")

                # Also try FrontCamera if available
                elif "FrontCamera" in self.drone.sensors:
                    cam = self.drone.sensors["FrontCamera"]
                    if "scene_camera" in cam:
                        self.client.subscribe(cam["scene_camera"], on_rgb_image)
                        print(f"    Subscribed to FrontCamera RGB")
                    if "depth_camera" in cam:
                        self.client.subscribe(cam["depth_camera"], on_depth_image)
                        print(f"    Subscribed to FrontCamera Depth")
                else:
                    print("  Warning: No recognized camera found in sensors")

            # Subscribe to pose/odometry via robot_info
            if hasattr(self.drone, 'robot_info'):
                print(f"  Available robot_info: {list(self.drone.robot_info.keys())}")
                if "actual_pose" in self.drone.robot_info:
                    self.client.subscribe(self.drone.robot_info["actual_pose"], on_pose)
                    print(f"    Subscribed to actual_pose for odometry")
            else:
                print("  Warning: drone.robot_info not available")

        except Exception as e:
            print(f"  Warning: Could not set up subscriptions: {e}")
            import traceback
            traceback.print_exc()

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
        # Odometry is published via subscription callback, not polling
        # Just check if we have data from the subscription
        with self.odom_lock:
            if self.latest_odom is not None:
                self.pub_odom.put(self.latest_odom.serialize())
                self.stats['odom_messages'] += 1
                self.latest_odom = None

    def _publish_rgb(self):
        """Publish RGB image to Zenoh."""
        try:
            # Check if we have subscribed image data
            with self.rgb_lock:
                if self.latest_rgb is not None:
                    img_data = self.latest_rgb
                    self.latest_rgb = None  # Consume it
                else:
                    img_data = None

            if img_data is not None:
                # Convert to our format and publish
                if isinstance(img_data, np.ndarray):
                    image_data = ImageData.from_numpy(img_data, ImageEncoding.BGR8)
                    self.pub_rgb.put(image_data.serialize())
                    self.stats['rgb_frames'] += 1

        except Exception as e:
            if self.stats['rgb_frames'] == 0:
                print(f"Error publishing RGB: {e}")

    def _publish_depth(self):
        """Publish depth image to Zenoh."""
        try:
            with self.depth_lock:
                if self.latest_depth is not None:
                    img_data = self.latest_depth
                    self.latest_depth = None
                else:
                    img_data = None

            if img_data is not None:
                if isinstance(img_data, np.ndarray):
                    # Normalize depth to 0-255 if needed
                    if img_data.dtype == np.float32:
                        max_depth = 100.0
                        depth_normalized = np.clip(img_data / max_depth * 255, 0, 255).astype(np.uint8)
                    else:
                        depth_normalized = img_data

                    image_data = ImageData.from_numpy(depth_normalized, ImageEncoding.GRAY8)
                    self.pub_depth.put(image_data.serialize())
                    self.stats['depth_frames'] += 1

        except Exception as e:
            if self.stats['depth_frames'] == 0:
                print(f"Error publishing depth: {e}")

    async def _apply_command_async(self):
        """Apply current velocity command to AirSim (async version)."""
        try:
            with self.command_lock:
                # Check for command timeout
                if time.time() - self.last_command_time > self.config.command_timeout:
                    cmd = VelocityCommand.hover()
                else:
                    cmd = self.current_command

            # Debug: print velocity every 50 commands (once per second at 50Hz)
            if self.stats['commands_applied'] % 50 == 0:
                print(f"  Vel cmd: vx={cmd.vx:.2f}, vy={cmd.vy:.2f}, vz={cmd.vz:.2f}, yaw_rate={cmd.yaw_rate:.2f}")

            # Apply velocity command to AirSim using async API
            # Project AirSim uses NED frame: v_north, v_east, v_down
            # yaw_is_rate=True means yaw parameter is rad/s, False means absolute angle
            move_task = await self.drone.move_by_velocity_async(
                v_north=cmd.vx,
                v_east=cmd.vy,
                v_down=cmd.vz,
                duration=0.2,
                yaw_is_rate=True,
                yaw=cmd.yaw_rate
            )
            # Don't await the task - let it run and be replaced by next command

            self.stats['commands_applied'] += 1

        except Exception as e:
            if self.stats['commands_applied'] == 0:
                print(f"Error applying command: {e}")

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

    async def _command_loop_async(self):
        """Command application loop (async)."""
        rate = 50.0  # 50 Hz command rate
        period = 1.0 / rate
        while self.running:
            start = time.time()
            await self._apply_command_async()
            elapsed = time.time() - start
            sleep_time = period - elapsed
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)

    async def run_async(self):
        """Main async run loop."""
        if not await self.connect_airsim():
            return False

        if not self.connect_zenoh():
            return False

        self.running = True
        print("\nBridge running. Press Ctrl+C to stop.\n")

        # Start sensor threads (synchronous - just publishing)
        threads = []

        odom_thread = threading.Thread(
            target=self._sensor_loop,
            args=(self._publish_odometry, self.config.odom_rate),
            daemon=True
        )
        threads.append(odom_thread)

        rgb_thread = threading.Thread(
            target=self._sensor_loop,
            args=(self._publish_rgb, self.config.rgb_rate),
            daemon=True
        )
        threads.append(rgb_thread)

        depth_thread = threading.Thread(
            target=self._sensor_loop,
            args=(self._publish_depth, self.config.depth_rate),
            daemon=True
        )
        threads.append(depth_thread)

        for t in threads:
            t.start()

        # Run command loop in async context
        cmd_task = asyncio.create_task(self._command_loop_async())

        # Print statistics periodically
        try:
            while self.running:
                await asyncio.sleep(5.0)
                self._print_stats()
        except asyncio.CancelledError:
            pass

        cmd_task.cancel()
        await self.shutdown_async()
        return True

    def _print_stats(self):
        """Print bridge statistics."""
        print(f"Stats: RGB={self.stats['rgb_frames']} | "
              f"Depth={self.stats['depth_frames']} | "
              f"Odom={self.stats['odom_messages']} | "
              f"CmdRx={self.stats['commands_received']} | "
              f"CmdTx={self.stats['commands_applied']}")

    async def shutdown_async(self):
        """Clean shutdown."""
        print("\nShutting down...")
        self.running = False

        # Hover and disarm
        if self.drone:
            try:
                # Send hover/stop command
                await self.drone.move_by_velocity_async(0, 0, 0, duration=0.1)
                await asyncio.sleep(0.5)

                land_task = await self.drone.land_async()
                await land_task

                self.drone.disarm()
                self.drone.disable_api_control()
            except Exception as e:
                print(f"Error during shutdown: {e}")

        # Disconnect client
        if self.client:
            try:
                self.client.disconnect()
            except Exception:
                pass

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
    parser.add_argument('--drone-name', type=str, default='Drone1',
                       help='Drone name in Project AirSim')
    parser.add_argument('--scene', type=str, default='scene_basic_drone.jsonc',
                       help='Scene file to load (default: scene_basic_drone.jsonc)')

    # Rate configuration
    parser.add_argument('--rgb-rate', type=float, default=30.0,
                       help='RGB image publish rate (Hz)')
    parser.add_argument('--depth-rate', type=float, default=30.0,
                       help='Depth image publish rate (Hz)')
    parser.add_argument('--odom-rate', type=float, default=100.0,
                       help='Odometry publish rate (Hz)')

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
        drone_name=args.drone_name,
        scene_file=args.scene,
        rgb_rate=args.rgb_rate,
        depth_rate=args.depth_rate,
        odom_rate=args.odom_rate,
        auto_takeoff=args.auto_takeoff,
        takeoff_altitude=args.altitude,
    )

    bridge = AirSimZenohBridge(config)

    # Handle signals
    def signal_handler(sig, frame):
        bridge.running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Run async main loop
    try:
        success = asyncio.run(bridge.run_async())
    except KeyboardInterrupt:
        success = True

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
