"""
Data types for Zenoh communication matching the C++ serialization format.

All binary formats use little-endian byte order to match the C++ implementation.
"""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional
import numpy as np


class ImageEncoding(IntEnum):
    """Image encoding types matching C++ ImageData::Encoding"""
    BGR8 = 0
    RGB8 = 1
    GRAY8 = 2
    BGRA8 = 3
    RGBA8 = 4


@dataclass
class ImageData:
    """
    Image data matching C++ ImageData serialization.

    Binary format (little-endian):
        [width:4][height:4][channels:4][encoding:4][pixel_data:variable]
        Header: 16 bytes
    """
    width: int
    height: int
    channels: int
    encoding: ImageEncoding
    data: bytes

    HEADER_SIZE = 16

    def serialize(self) -> bytes:
        """Serialize to binary format matching C++ implementation."""
        header = struct.pack('<IIII',
                            self.width,
                            self.height,
                            self.channels,
                            int(self.encoding))
        return header + self.data

    @classmethod
    def deserialize(cls, payload: bytes) -> 'ImageData':
        """Deserialize from binary format."""
        if len(payload) < cls.HEADER_SIZE:
            raise ValueError(f"Payload too small: {len(payload)} < {cls.HEADER_SIZE}")

        width, height, channels, encoding = struct.unpack('<IIII', payload[:cls.HEADER_SIZE])
        data = payload[cls.HEADER_SIZE:]

        expected_size = width * height * channels
        if len(data) != expected_size:
            raise ValueError(f"Data size mismatch: {len(data)} != {expected_size}")

        return cls(
            width=width,
            height=height,
            channels=channels,
            encoding=ImageEncoding(encoding),
            data=data
        )

    @classmethod
    def from_numpy(cls, img: np.ndarray, encoding: ImageEncoding = ImageEncoding.BGR8) -> 'ImageData':
        """Create ImageData from numpy array (OpenCV format)."""
        if img.ndim == 2:
            height, width = img.shape
            channels = 1
        else:
            height, width, channels = img.shape

        # Ensure contiguous memory layout
        if not img.flags['C_CONTIGUOUS']:
            img = np.ascontiguousarray(img)

        return cls(
            width=width,
            height=height,
            channels=channels,
            encoding=encoding,
            data=img.tobytes()
        )

    def to_numpy(self) -> np.ndarray:
        """Convert to numpy array (OpenCV format)."""
        if self.channels == 1:
            shape = (self.height, self.width)
        else:
            shape = (self.height, self.width, self.channels)

        return np.frombuffer(self.data, dtype=np.uint8).reshape(shape)


@dataclass
class Odometry:
    """
    Odometry data matching C++ Odometry serialization.

    Binary format (little-endian):
        [x:f32][y:f32][z:f32][roll:f32][pitch:f32][yaw:f32][timestamp_us:u64]
        Total: 32 bytes
    """
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    timestamp_us: int

    SERIALIZED_SIZE = 32
    FORMAT = '<ffffffQ'  # 6 floats + 1 uint64

    def serialize(self) -> bytes:
        """Serialize to binary format matching C++ implementation."""
        return struct.pack(self.FORMAT,
                          self.x, self.y, self.z,
                          self.roll, self.pitch, self.yaw,
                          self.timestamp_us)

    @classmethod
    def deserialize(cls, payload: bytes) -> 'Odometry':
        """Deserialize from binary format."""
        if len(payload) != cls.SERIALIZED_SIZE:
            raise ValueError(f"Invalid payload size: {len(payload)} != {cls.SERIALIZED_SIZE}")

        x, y, z, roll, pitch, yaw, timestamp_us = struct.unpack(cls.FORMAT, payload)
        return cls(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, timestamp_us=timestamp_us)


class CommandPriority(IntEnum):
    """Command priority levels matching C++ VelocityCommand::Priority"""
    LOW = 0       # Waypoint following
    NORMAL = 1    # Manual override
    HIGH = 2      # Reactive avoidance
    CRITICAL = 3  # Emergency stop


class CommandSource(IntEnum):
    """Command source types matching C++ VelocityCommand::Source"""
    UNKNOWN = 0
    WAYPOINT_MANAGER = 1
    OBSTACLE_AVOIDANCE = 2
    MANUAL_CONTROL = 3
    EMERGENCY = 4
    COMMAND_ARBITER = 5


@dataclass
class VelocityCommand:
    """
    Velocity command matching C++ VelocityCommand serialization.

    Binary format (little-endian):
        [vx:f32][vy:f32][vz:f32][yaw_rate:f32][priority:u8][source:u8][padding:2][timestamp_us:u64]
        Total: 28 bytes

    Velocity is in body frame: vx=forward, vy=right, vz=down (NED)
    """
    vx: float
    vy: float
    vz: float
    yaw_rate: float
    priority: CommandPriority
    source: CommandSource
    timestamp_us: int

    SERIALIZED_SIZE = 28
    FORMAT = '<ffffBBxxQ'  # 4 floats + 2 uint8 + 2 padding + 1 uint64

    def serialize(self) -> bytes:
        """Serialize to binary format matching C++ implementation."""
        return struct.pack(self.FORMAT,
                          self.vx, self.vy, self.vz, self.yaw_rate,
                          int(self.priority), int(self.source),
                          self.timestamp_us)

    @classmethod
    def deserialize(cls, payload: bytes) -> 'VelocityCommand':
        """Deserialize from binary format."""
        if len(payload) != cls.SERIALIZED_SIZE:
            raise ValueError(f"Invalid payload size: {len(payload)} != {cls.SERIALIZED_SIZE}")

        vx, vy, vz, yaw_rate, priority, source, timestamp_us = struct.unpack(cls.FORMAT, payload)
        return cls(
            vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate,
            priority=CommandPriority(priority),
            source=CommandSource(source),
            timestamp_us=timestamp_us
        )

    @classmethod
    def hover(cls, timestamp_us: int = 0) -> 'VelocityCommand':
        """Create a hover command (zero velocity)."""
        return cls(
            vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0,
            priority=CommandPriority.NORMAL,
            source=CommandSource.UNKNOWN,
            timestamp_us=timestamp_us
        )

    @classmethod
    def emergency_stop(cls, timestamp_us: int = 0) -> 'VelocityCommand':
        """Create an emergency stop command."""
        return cls(
            vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0,
            priority=CommandPriority.CRITICAL,
            source=CommandSource.EMERGENCY,
            timestamp_us=timestamp_us
        )


@dataclass
class Waypoint:
    """
    Single waypoint matching C++ Waypoint serialization.

    Binary format (little-endian):
        [x:f32][y:f32][z:f32][yaw:f32][speed:f32][id:u32][flags:u8][padding:3]
        Total: 28 bytes
    """
    x: float
    y: float
    z: float
    yaw: float = 0.0
    speed: float = 1.0
    id: int = 0
    flags: int = 0

    # Flags as class attributes (matching C++ nested enum)
    NONE = 0
    HOLD_YAW = 1 << 0    # Maintain specified yaw during approach
    HOVER = 1 << 1       # Hover at waypoint briefly
    FINAL = 1 << 2       # Last waypoint in sequence

    SERIALIZED_SIZE = 28
    FORMAT = '<fffffIBxxx'  # 5 floats + 1 uint32 + 1 uint8 + 3 padding

    def serialize(self) -> bytes:
        """Serialize to binary format matching C++ implementation."""
        return struct.pack(self.FORMAT,
                          self.x, self.y, self.z, self.yaw, self.speed,
                          self.id, self.flags)

    @classmethod
    def deserialize(cls, payload: bytes) -> 'Waypoint':
        """Deserialize from binary format."""
        if len(payload) < cls.SERIALIZED_SIZE:
            raise ValueError(f"Invalid payload size: {len(payload)} < {cls.SERIALIZED_SIZE}")

        x, y, z, yaw, speed, wp_id, flags = struct.unpack(cls.FORMAT, payload[:cls.SERIALIZED_SIZE])
        return cls(x=x, y=y, z=z, yaw=yaw, speed=speed, id=wp_id, flags=flags)


@dataclass
class WaypointList:
    """
    Waypoint list matching C++ WaypointList serialization.

    Binary format (little-endian):
        [count:u32][timestamp_us:u64][waypoints:28*count]
    """
    waypoints: list
    timestamp_us: int = 0

    def serialize(self) -> bytes:
        """Serialize to binary format matching C++ implementation."""
        # Header: count (4 bytes) + timestamp (8 bytes), then waypoints
        data = struct.pack('<IQ', len(self.waypoints), self.timestamp_us)
        for wp in self.waypoints:
            data += wp.serialize()
        return data

    @classmethod
    def deserialize(cls, payload: bytes) -> 'WaypointList':
        """Deserialize from binary format."""
        header_size = 4 + 8  # count + timestamp
        if len(payload) < header_size:
            raise ValueError(f"Payload too small: {len(payload)} < {header_size}")

        count, timestamp_us = struct.unpack('<IQ', payload[:header_size])
        offset = header_size
        waypoints = []

        for _ in range(count):
            wp = Waypoint.deserialize(payload[offset:offset + Waypoint.SERIALIZED_SIZE])
            waypoints.append(wp)
            offset += Waypoint.SERIALIZED_SIZE

        return cls(waypoints=waypoints, timestamp_us=timestamp_us)
