#!/usr/bin/env python3
"""
Pillar Corridor Mission - Test obstacle avoidance through a corridor with pillars.

This script publishes waypoints that guide the drone straight through a corridor
while simulating proximity sensor data based on pillar positions. The obstacle
avoidance system should react and navigate around the obstacles.

Corridor layout (30m long x 6m wide):
    START                                         GOAL
      ↓                                             ↓
    [wall]     P1    P2          P3    P4        [wall]
      X=0            X=8         X=16   X=24      X=30

Pillar positions (1m diameter):
  P1: (8, -1.5, -2)  - left
  P2: (8, 1.5, -2)   - right (in direct path)
  P3: (16, -1.5, -2) - left (in direct path)
  P4: (24, 1.5, -2)  - right

Usage:
    python pillar_corridor_mission.py --connect tcp/192.168.1.10:7447
"""

import argparse
import signal
import sys
import time
import threading
import math
import struct
from typing import Optional, List, Tuple

import zenoh

from data_types import (
    Odometry, Waypoint, WaypointList,
    CommandPriority, CommandSource
)


# Pillar positions (x, y, z) in NED coordinates
PILLARS = [
    (8.0, -1.5, -2.0),   # P1: left side
    (8.0, 1.5, -2.0),    # P2: right side (blocks Y=0 path)
    (16.0, -1.5, -2.0),  # P3: left side (blocks Y=0 path)
    (24.0, 1.5, -2.0),   # P4: right side
]

PILLAR_RADIUS = 0.5  # meters (1m diameter)
CORRIDOR_WIDTH = 6.0  # meters
CORRIDOR_LENGTH = 30.0  # meters


class ProximityData:
    """6-direction proximity sensor data."""

    SERIALIZED_SIZE = 32  # 6 floats + 1 uint64

    def __init__(self, front=999.0, back=999.0, left=999.0, right=999.0,
                 up=999.0, down=999.0, timestamp_us=0):
        self.front = front
        self.back = back
        self.left = left
        self.right = right
        self.up = up
        self.down = down
        self.timestamp_us = timestamp_us

    def serialize(self) -> bytes:
        """Serialize to bytes (little-endian)."""
        return struct.pack('<ffffffQ',
                          self.front, self.back, self.left, self.right,
                          self.up, self.down, self.timestamp_us)

    @classmethod
    def clear(cls, timestamp_us: int = 0) -> 'ProximityData':
        """Create clear readings (no obstacles)."""
        return cls(999.0, 999.0, 999.0, 999.0, 999.0, 999.0, timestamp_us)


class PillarCorridorMission:
    """Run obstacle avoidance test through pillar corridor."""

    def __init__(self, connect_endpoint: str, robot_id: str = "drone"):
        self.connect_endpoint = connect_endpoint
        self.robot_id = robot_id
        self.running = False
        self.session: Optional[zenoh.Session] = None

        # Current state
        self.current_odom: Optional[Odometry] = None
        self.odom_lock = threading.Lock()

        # Topics
        self.topic_odom = f"robot/{robot_id}/sensor/state/odom"
        self.topic_waypoints = f"robot/{robot_id}/cmd/waypoints"
        self.topic_proximity = f"robot/{robot_id}/sensor/proximity"

    def connect(self) -> bool:
        """Connect to Zenoh."""
        try:
            print(f"Connecting to Zenoh at {self.connect_endpoint}...")

            config = zenoh.Config()
            config.insert_json5("connect/endpoints", f'["{self.connect_endpoint}"]')

            self.session = zenoh.open(config)
            print("Connected to Zenoh successfully")

            # Subscribe to odometry
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

    def get_position(self) -> Tuple[float, float, float]:
        """Get current position (x, y, z)."""
        with self.odom_lock:
            if self.current_odom:
                return (self.current_odom.x, self.current_odom.y, self.current_odom.z)
        return (0, 0, 0)

    def compute_proximity(self, pos: Tuple[float, float, float]) -> ProximityData:
        """Compute proximity readings based on pillar positions."""
        x, y, z = pos
        timestamp = int(time.time() * 1_000_000)

        prox = ProximityData.clear(timestamp)

        # Check distance to each pillar
        for px, py, pz in PILLARS:
            # Distance to pillar center in XY plane (pillars are vertical)
            dx = px - x
            dy = py - y
            dist_xy = math.sqrt(dx*dx + dy*dy)

            # Distance to pillar surface
            dist_surface = dist_xy - PILLAR_RADIUS
            if dist_surface < 0:
                dist_surface = 0.01  # Inside pillar

            # Determine which direction this pillar is in
            angle = math.atan2(dy, dx)  # Angle from drone to pillar

            # Map angle to direction (assuming drone faces +X)
            # Front: angle near 0
            # Back: angle near ±π
            # Left: angle near -π/2
            # Right: angle near +π/2

            if abs(angle) < math.pi/4:  # Front
                prox.front = min(prox.front, dist_surface)
            elif abs(angle) > 3*math.pi/4:  # Back
                prox.back = min(prox.back, dist_surface)
            elif angle > 0:  # Right
                prox.right = min(prox.right, dist_surface)
            else:  # Left
                prox.left = min(prox.left, dist_surface)

        # Floor and ceiling
        altitude = -z  # NED: z is down, altitude is up
        prox.down = max(0.1, altitude)  # Distance to ground
        prox.up = 999.0  # No ceiling

        return prox

    def publish_waypoints(self):
        """Publish corridor waypoints."""
        # Use current altitude to avoid unexpected descent/ascent
        current_z = -5.0  # Default safe altitude (5m up in NED)
        with self.odom_lock:
            if self.current_odom:
                current_z = self.current_odom.z

        print(f"Using waypoint altitude: z={current_z:.1f}")

        waypoints = [
            Waypoint(x=10.0, y=0.0, z=current_z, yaw=0.0, speed=1.0,
                    flags=Waypoint.HOLD_YAW, id=1),
            Waypoint(x=20.0, y=0.0, z=current_z, yaw=0.0, speed=1.0,
                    flags=Waypoint.HOLD_YAW, id=2),
            Waypoint(x=30.0, y=0.0, z=current_z, yaw=0.0, speed=1.0,
                    flags=Waypoint.HOLD_YAW, id=3),
        ]

        wpl = WaypointList(waypoints)
        self.session.put(self.topic_waypoints, wpl.serialize())
        print(f"Published {len(waypoints)} waypoints through corridor")

    def run(self, duration: float = 60.0, proximity_rate: float = 20.0):
        """Run the mission."""
        self.running = True

        print("\n" + "="*60)
        print("Pillar Corridor Mission")
        print("="*60)
        print(f"\nCorridor: {CORRIDOR_LENGTH}m long, {CORRIDOR_WIDTH}m wide")
        print(f"Pillars at: {PILLARS}")
        print(f"Goal: Navigate from X=0 to X=30 without collision")
        print(f"Duration: {duration}s, Proximity rate: {proximity_rate}Hz")
        print("="*60 + "\n")

        # Wait for odometry
        print("Waiting for odometry...")
        for _ in range(50):
            if self.current_odom:
                break
            time.sleep(0.1)

        if not self.current_odom:
            print("WARNING: No odometry received, using origin")

        pos = self.get_position()
        print(f"Starting position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")

        # Publish waypoints
        self.publish_waypoints()

        # Main loop: publish proximity data
        period = 1.0 / proximity_rate
        start_time = time.time()
        last_status_time = start_time

        print("\nRunning mission (publishing proximity data)...")
        print("Press Ctrl+C to stop.\n")

        while self.running and (time.time() - start_time) < duration:
            pos = self.get_position()

            # Compute and publish proximity
            prox = self.compute_proximity(pos)
            self.session.put(self.topic_proximity, prox.serialize())

            # Status update every 2 seconds
            if time.time() - last_status_time > 2.0:
                last_status_time = time.time()
                min_dist = min(prox.front, prox.back, prox.left, prox.right)
                print(f"  Pos: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) "
                      f"Min proximity: {min_dist:.2f}m")

                # Check if goal reached
                if pos[0] >= CORRIDOR_LENGTH - 1.0:
                    print("\n*** GOAL REACHED! ***")
                    break

            time.sleep(period)

        # Final status
        pos = self.get_position()
        print(f"\nMission ended. Final position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")

        if pos[0] >= CORRIDOR_LENGTH - 1.0:
            print("RESULT: SUCCESS - Reached end of corridor")
        else:
            print(f"RESULT: INCOMPLETE - Only reached X={pos[0]:.1f}")

    def stop(self):
        """Stop the mission."""
        self.running = False
        if self.session:
            try:
                self.session.close()
            except Exception:
                pass
            self.session = None


def main():
    parser = argparse.ArgumentParser(description="Pillar corridor obstacle avoidance mission")
    parser.add_argument("--connect", required=True, help="Zenoh endpoint (e.g., tcp/192.168.1.10:7447)")
    parser.add_argument("--robot-id", default="drone", help="Robot ID (default: drone)")
    parser.add_argument("--duration", type=float, default=60.0, help="Mission duration in seconds")
    parser.add_argument("--rate", type=float, default=20.0, help="Proximity publish rate in Hz")

    args = parser.parse_args()

    mission = PillarCorridorMission(args.connect, args.robot_id)

    def signal_handler(sig, frame):
        print("\nInterrupted, stopping...")
        mission.stop()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if not mission.connect():
        return 1

    try:
        mission.run(args.duration, args.rate)
    except KeyboardInterrupt:
        print("\nInterrupted...")
    finally:
        mission.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
