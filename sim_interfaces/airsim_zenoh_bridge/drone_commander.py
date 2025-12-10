#!/usr/bin/env python3
"""
Drone Commander - Send velocity commands to the drone via Zenoh.

This script connects to the AirSim Zenoh bridge and sends velocity commands,
allowing you to fly the drone around interactively or run scripted missions.

Usage:
    # Interactive mode (keyboard control)
    python drone_commander.py --connect tcp/192.168.1.10:7447

    # Run a scripted mission
    python drone_commander.py --connect tcp/192.168.1.10:7447 --mission square

    # Single command
    python drone_commander.py --connect tcp/192.168.1.10:7447 --cmd "forward 2"
"""

import argparse
import signal
import sys
import time
import threading
from typing import Optional

import zenoh

from data_types import (
    Odometry, VelocityCommand,
    CommandPriority, CommandSource
)


class DroneCommander:
    """Send commands to drone via Zenoh."""

    def __init__(self, connect_endpoint: str, robot_id: str = "drone"):
        self.connect_endpoint = connect_endpoint
        self.robot_id = robot_id
        self.running = False
        self.session: Optional[zenoh.Session] = None

        # Current state from odometry
        self.current_odom: Optional[Odometry] = None
        self.odom_lock = threading.Lock()

        # Topics
        self.topic_odom = f"robot/{robot_id}/sensor/state/odom"
        self.topic_cmd_vel = f"robot/{robot_id}/cmd/velocity"

    def connect(self) -> bool:
        """Connect to Zenoh."""
        try:
            print(f"Connecting to Zenoh at {self.connect_endpoint}...")

            config = zenoh.Config()
            config.insert_json5("connect/endpoints", f'["{self.connect_endpoint}"]')

            self.session = zenoh.open(config)
            print("Connected to Zenoh successfully")

            # Subscribe to odometry for feedback
            self.session.declare_subscriber(self.topic_odom, self._on_odom)
            print(f"Subscribed to odometry: {self.topic_odom}")

            return True

        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def _on_odom(self, sample: zenoh.Sample):
        """Handle odometry updates."""
        try:
            odom = Odometry.deserialize(bytes(sample.payload))
            with self.odom_lock:
                self.current_odom = odom
        except Exception as e:
            print(f"Error parsing odometry: {e}")

    def get_position(self) -> tuple:
        """Get current position (x, y, z)."""
        with self.odom_lock:
            if self.current_odom:
                return (self.current_odom.x, self.current_odom.y, self.current_odom.z)
        return (0, 0, 0)

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0,
                     priority: CommandPriority = CommandPriority.NORMAL,
                     source: CommandSource = CommandSource.MANUAL_CONTROL):
        """Send a velocity command."""
        cmd = VelocityCommand(
            vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate,
            priority=priority, source=source,
            timestamp_us=int(time.time() * 1_000_000)
        )
        self.session.put(self.topic_cmd_vel, cmd.serialize())

    def hover(self):
        """Send hover command (zero velocity)."""
        self.send_velocity(0, 0, 0, 0)

    def move_for_duration(self, vx: float, vy: float, vz: float, yaw_rate: float,
                          duration: float, rate: float = 20.0):
        """Send velocity commands for a specified duration."""
        period = 1.0 / rate
        end_time = time.time() + duration

        while time.time() < end_time and self.running:
            self.send_velocity(vx, vy, vz, yaw_rate)
            time.sleep(period)

        # Stop after movement
        self.hover()

    def run_mission_square(self, side_length: float = 5.0, speed: float = 1.0):
        """Fly a square pattern."""
        duration = side_length / speed

        print(f"\nFlying square pattern: {side_length}m sides at {speed}m/s")
        print(f"Each leg takes {duration:.1f}s\n")

        legs = [
            ("Forward (North)", speed, 0, 0, 0),
            ("Right (East)", 0, speed, 0, 0),
            ("Backward (South)", -speed, 0, 0, 0),
            ("Left (West)", 0, -speed, 0, 0),
        ]

        for name, vx, vy, vz, yaw in legs:
            if not self.running:
                break
            pos = self.get_position()
            print(f"  {name}: Starting at ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
            self.move_for_duration(vx, vy, vz, yaw, duration)
            time.sleep(0.5)  # Brief pause between legs

        pos = self.get_position()
        print(f"\nSquare complete. Final position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")

    def run_mission_circle(self, radius: float = 3.0, duration: float = 10.0):
        """Fly a circular pattern using forward velocity + yaw rate."""
        # For a circle: forward_velocity = radius * yaw_rate
        # Full circle in 'duration' seconds means yaw_rate = 2*pi / duration
        import math
        yaw_rate = 2 * math.pi / duration
        forward_velocity = radius * yaw_rate

        print(f"\nFlying circle: radius={radius}m, duration={duration}s")
        print(f"  Forward velocity: {forward_velocity:.2f} m/s")
        print(f"  Yaw rate: {yaw_rate:.2f} rad/s ({math.degrees(yaw_rate):.1f} deg/s)\n")

        self.move_for_duration(forward_velocity, 0, 0, yaw_rate, duration)

        pos = self.get_position()
        print(f"\nCircle complete. Final position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")

    def run_mission_up_down(self, height: float = 5.0, speed: float = 1.0):
        """Move up and down."""
        duration = height / speed

        print(f"\nMoving up {height}m then down at {speed}m/s")

        pos = self.get_position()
        print(f"  Starting at z={pos[2]:.1f}")

        print("  Going up...")
        self.move_for_duration(0, 0, -speed, 0, duration)  # NED: negative z is up

        pos = self.get_position()
        print(f"  At z={pos[2]:.1f}, going down...")

        self.move_for_duration(0, 0, speed, 0, duration)  # NED: positive z is down

        pos = self.get_position()
        print(f"\nUp/down complete. Final z={pos[2]:.1f}")

    def interactive_mode(self):
        """Interactive keyboard control mode."""
        print("\n" + "="*60)
        print("Interactive Drone Control")
        print("="*60)
        print("\nCommands:")
        print("  w/s     - Forward/Backward")
        print("  a/d     - Left/Right (strafe)")
        print("  q/e     - Rotate left/right (yaw)")
        print("  r/f     - Up/Down")
        print("  space   - Hover (stop)")
        print("  1       - Run square mission")
        print("  2       - Run circle mission")
        print("  3       - Run up/down mission")
        print("  p       - Print current position")
        print("  x/ESC   - Exit")
        print("\nSpeed: 1.0 m/s, Yaw: 0.5 rad/s")
        print("="*60 + "\n")

        speed = 1.0
        yaw_speed = 0.5

        # For non-blocking input
        try:
            import msvcrt  # Windows
            def get_key():
                if msvcrt.kbhit():
                    return msvcrt.getch().decode('utf-8', errors='ignore').lower()
                return None
        except ImportError:
            import select
            import tty
            import termios
            old_settings = termios.tcgetattr(sys.stdin)

            def get_key():
                try:
                    tty.setraw(sys.stdin.fileno())
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        return sys.stdin.read(1).lower()
                finally:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                return None

        current_cmd = (0, 0, 0, 0)  # vx, vy, vz, yaw

        try:
            while self.running:
                key = get_key()

                if key:
                    if key == 'w':
                        current_cmd = (speed, 0, 0, 0)
                        print("Forward")
                    elif key == 's':
                        current_cmd = (-speed, 0, 0, 0)
                        print("Backward")
                    elif key == 'a':
                        current_cmd = (0, -speed, 0, 0)
                        print("Left")
                    elif key == 'd':
                        current_cmd = (0, speed, 0, 0)
                        print("Right")
                    elif key == 'q':
                        current_cmd = (0, 0, 0, -yaw_speed)
                        print("Rotate Left")
                    elif key == 'e':
                        current_cmd = (0, 0, 0, yaw_speed)
                        print("Rotate Right")
                    elif key == 'r':
                        current_cmd = (0, 0, -speed, 0)
                        print("Up")
                    elif key == 'f':
                        current_cmd = (0, 0, speed, 0)
                        print("Down")
                    elif key == ' ':
                        current_cmd = (0, 0, 0, 0)
                        print("Hover")
                    elif key == '1':
                        print("\nStarting square mission...")
                        self.run_mission_square()
                        current_cmd = (0, 0, 0, 0)
                    elif key == '2':
                        print("\nStarting circle mission...")
                        self.run_mission_circle()
                        current_cmd = (0, 0, 0, 0)
                    elif key == '3':
                        print("\nStarting up/down mission...")
                        self.run_mission_up_down()
                        current_cmd = (0, 0, 0, 0)
                    elif key == 'p':
                        pos = self.get_position()
                        print(f"Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
                    elif key in ('x', '\x1b'):  # x or ESC
                        print("\nExiting...")
                        break

                # Send current command
                self.send_velocity(*current_cmd)
                time.sleep(0.05)  # 20 Hz

        except Exception as e:
            print(f"Interactive mode error: {e}")

        finally:
            self.hover()

    def run_single_command(self, cmd_str: str):
        """Parse and execute a single command string."""
        parts = cmd_str.lower().split()
        if not parts:
            return

        cmd = parts[0]
        args = parts[1:] if len(parts) > 1 else []

        duration = float(args[0]) if args else 2.0
        speed = float(args[1]) if len(args) > 1 else 1.0

        print(f"Executing: {cmd} for {duration}s at {speed}m/s")

        if cmd == "forward":
            self.move_for_duration(speed, 0, 0, 0, duration)
        elif cmd == "backward":
            self.move_for_duration(-speed, 0, 0, 0, duration)
        elif cmd == "left":
            self.move_for_duration(0, -speed, 0, 0, duration)
        elif cmd == "right":
            self.move_for_duration(0, speed, 0, 0, duration)
        elif cmd == "up":
            self.move_for_duration(0, 0, -speed, 0, duration)
        elif cmd == "down":
            self.move_for_duration(0, 0, speed, 0, duration)
        elif cmd == "rotate_left":
            self.move_for_duration(0, 0, 0, -speed, duration)
        elif cmd == "rotate_right":
            self.move_for_duration(0, 0, 0, speed, duration)
        elif cmd == "hover":
            self.hover()
            time.sleep(duration)
        else:
            print(f"Unknown command: {cmd}")
            print("Available: forward, backward, left, right, up, down, rotate_left, rotate_right, hover")

    def disconnect(self):
        """Disconnect from Zenoh."""
        self.hover()  # Stop the drone first
        if self.session:
            self.session.close()
        print("Disconnected")


def main():
    parser = argparse.ArgumentParser(
        description="Drone Commander - Control drone via Zenoh",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Interactive keyboard control
  python drone_commander.py --connect tcp/192.168.1.10:7447

  # Run square mission
  python drone_commander.py --connect tcp/192.168.1.10:7447 --mission square

  # Single command: forward for 3 seconds at 1.5 m/s
  python drone_commander.py --connect tcp/192.168.1.10:7447 --cmd "forward 3 1.5"
        """
    )

    parser.add_argument('--connect', type=str, required=True,
                       help='Zenoh endpoint (e.g., tcp/192.168.1.10:7447)')
    parser.add_argument('--robot-id', type=str, default='drone',
                       help='Robot ID for topics')
    parser.add_argument('--mission', type=str, choices=['square', 'circle', 'updown'],
                       help='Run a predefined mission')
    parser.add_argument('--cmd', type=str,
                       help='Run a single command (e.g., "forward 2")')

    args = parser.parse_args()

    commander = DroneCommander(args.connect, args.robot_id)

    def signal_handler(sig, frame):
        commander.running = False
        commander.hover()
        print("\nStopping...")

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if not commander.connect():
        sys.exit(1)

    commander.running = True

    # Wait a moment for odometry to start flowing
    time.sleep(0.5)

    try:
        if args.mission:
            if args.mission == 'square':
                commander.run_mission_square()
            elif args.mission == 'circle':
                commander.run_mission_circle()
            elif args.mission == 'updown':
                commander.run_mission_up_down()
        elif args.cmd:
            commander.run_single_command(args.cmd)
        else:
            # Interactive mode
            commander.interactive_mode()

    finally:
        commander.running = False
        commander.disconnect()


if __name__ == '__main__':
    main()
