#!/usr/bin/env python3
"""
Test script for verifying AirSim Zenoh Bridge connectivity.

Run this on the Linux workstation to test connectivity with the
bridge running on the Windows workstation.

Usage:
    python test_bridge.py --connect tcp/192.168.1.10:7447
    python test_bridge.py --connect tcp/192.168.1.10:7447 --send-commands
"""

import argparse
import signal
import sys
import time
import threading
from dataclasses import dataclass
from typing import Optional

import zenoh

from data_types import ImageData, Odometry, VelocityCommand, CommandPriority, CommandSource


@dataclass
class Stats:
    """Statistics for received data."""
    rgb_frames: int = 0
    depth_frames: int = 0
    odom_messages: int = 0
    last_odom: Optional[Odometry] = None
    last_rgb_size: tuple = (0, 0)
    last_depth_size: tuple = (0, 0)
    start_time: float = 0.0


class BridgeTester:
    """Test client for the AirSim Zenoh Bridge."""

    def __init__(self, connect_endpoint: str, robot_id: str = "drone"):
        self.connect_endpoint = connect_endpoint
        self.robot_id = robot_id
        self.running = False
        self.stats = Stats()
        self.session: Optional[zenoh.Session] = None

        # Topic key expressions
        self.topic_rgb = f"robot/{robot_id}/sensor/camera/rgb"
        self.topic_depth = f"robot/{robot_id}/sensor/camera/depth"
        self.topic_odom = f"robot/{robot_id}/sensor/state/odom"
        self.topic_cmd_vel = f"robot/{robot_id}/cmd/velocity"

    def connect(self) -> bool:
        """Connect to the Zenoh bridge."""
        try:
            print(f"Connecting to Zenoh bridge at {self.connect_endpoint}...")

            config = zenoh.Config()
            config.insert_json5("connect/endpoints", f'["{self.connect_endpoint}"]')

            self.session = zenoh.open(config)
            print("Connected to Zenoh bridge successfully")
            return True

        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def _on_rgb(self, sample: zenoh.Sample):
        """Handle RGB image data."""
        try:
            img = ImageData.deserialize(bytes(sample.payload))
            self.stats.rgb_frames += 1
            self.stats.last_rgb_size = (img.width, img.height)
        except Exception as e:
            print(f"Error parsing RGB: {e}")

    def _on_depth(self, sample: zenoh.Sample):
        """Handle depth image data."""
        try:
            img = ImageData.deserialize(bytes(sample.payload))
            self.stats.depth_frames += 1
            self.stats.last_depth_size = (img.width, img.height)
        except Exception as e:
            print(f"Error parsing depth: {e}")

    def _on_odom(self, sample: zenoh.Sample):
        """Handle odometry data."""
        try:
            odom = Odometry.deserialize(bytes(sample.payload))
            self.stats.odom_messages += 1
            self.stats.last_odom = odom
        except Exception as e:
            print(f"Error parsing odometry: {e}")

    def subscribe_all(self):
        """Subscribe to all sensor topics."""
        print(f"\nSubscribing to topics:")
        print(f"  RGB:   {self.topic_rgb}")
        print(f"  Depth: {self.topic_depth}")
        print(f"  Odom:  {self.topic_odom}")

        self.session.declare_subscriber(self.topic_rgb, self._on_rgb)
        self.session.declare_subscriber(self.topic_depth, self._on_depth)
        self.session.declare_subscriber(self.topic_odom, self._on_odom)

    def send_test_command(self, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0, yaw_rate: float = 0.0):
        """Send a test velocity command."""
        cmd = VelocityCommand(
            vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate,
            priority=CommandPriority.NORMAL,
            source=CommandSource.MANUAL_CONTROL,
            timestamp_us=int(time.time() * 1_000_000)
        )
        self.session.put(self.topic_cmd_vel, cmd.serialize())
        print(f"Sent command: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw={yaw_rate:.2f}")

    def print_stats(self):
        """Print current statistics."""
        elapsed = time.time() - self.stats.start_time

        print(f"\n{'='*60}")
        print(f"Bridge Test Statistics (elapsed: {elapsed:.1f}s)")
        print(f"{'='*60}")

        # Calculate rates
        rgb_rate = self.stats.rgb_frames / elapsed if elapsed > 0 else 0
        depth_rate = self.stats.depth_frames / elapsed if elapsed > 0 else 0
        odom_rate = self.stats.odom_messages / elapsed if elapsed > 0 else 0

        print(f"RGB Frames:   {self.stats.rgb_frames:6d}  ({rgb_rate:5.1f} Hz)  Size: {self.stats.last_rgb_size}")
        print(f"Depth Frames: {self.stats.depth_frames:6d}  ({depth_rate:5.1f} Hz)  Size: {self.stats.last_depth_size}")
        print(f"Odom Messages:{self.stats.odom_messages:6d}  ({odom_rate:5.1f} Hz)")

        if self.stats.last_odom:
            o = self.stats.last_odom
            print(f"\nLast Odometry:")
            print(f"  Position: x={o.x:8.3f}, y={o.y:8.3f}, z={o.z:8.3f}")
            print(f"  Rotation: roll={o.roll:6.3f}, pitch={o.pitch:6.3f}, yaw={o.yaw:6.3f}")

        # Connection status
        if self.stats.odom_messages > 0:
            print(f"\n✓ Bridge connection: OK")
        else:
            print(f"\n✗ Bridge connection: NO DATA RECEIVED")
            print(f"  Check that:")
            print(f"  - The bridge is running on Windows")
            print(f"  - Project AirSim is running")
            print(f"  - Firewall port 7447 is open")
            print(f"  - The endpoint address is correct")

    def run(self, duration: float = 10.0, send_commands: bool = False):
        """Run the test for a specified duration."""
        if not self.connect():
            return False

        self.subscribe_all()
        self.stats.start_time = time.time()
        self.running = True

        print(f"\nListening for {duration} seconds...")
        if send_commands:
            print("Will send test commands every 2 seconds")

        command_interval = 2.0
        last_command_time = 0.0
        command_sequence = [
            (0.5, 0.0, 0.0, 0.0),   # Forward
            (0.0, 0.5, 0.0, 0.0),   # Right
            (-0.5, 0.0, 0.0, 0.0),  # Backward
            (0.0, -0.5, 0.0, 0.0),  # Left
            (0.0, 0.0, 0.0, 0.5),   # Rotate CW
            (0.0, 0.0, 0.0, 0.0),   # Hover
        ]
        cmd_idx = 0

        try:
            while self.running and (time.time() - self.stats.start_time) < duration:
                current_time = time.time()

                # Send test commands if enabled
                if send_commands and (current_time - last_command_time) >= command_interval:
                    vx, vy, vz, yaw = command_sequence[cmd_idx % len(command_sequence)]
                    self.send_test_command(vx, vy, vz, yaw)
                    cmd_idx += 1
                    last_command_time = current_time

                time.sleep(0.1)

        except KeyboardInterrupt:
            pass

        self.print_stats()
        self.session.close()
        return self.stats.odom_messages > 0


def main():
    parser = argparse.ArgumentParser(
        description="Test AirSim Zenoh Bridge connectivity",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument('--connect', type=str, required=True,
                       help='Zenoh endpoint to connect to (e.g., tcp/192.168.1.10:7447)')
    parser.add_argument('--robot-id', type=str, default='drone',
                       help='Robot ID for topic namespacing')
    parser.add_argument('--duration', type=float, default=10.0,
                       help='Test duration in seconds')
    parser.add_argument('--send-commands', action='store_true',
                       help='Send test velocity commands')

    args = parser.parse_args()

    tester = BridgeTester(args.connect, args.robot_id)

    # Handle signals
    def signal_handler(sig, frame):
        tester.running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    success = tester.run(args.duration, args.send_commands)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
