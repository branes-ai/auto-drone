#!/usr/bin/env python3
"""
Mission Recording Service

Records camera feeds from drone missions to video files.
Supports single camera and multi-view layouts.

Usage:
    # Record chase camera (cinematic 3rd person view)
    python record_mission.py --connect tcp/192.168.1.10:7447 \
        --camera chase --output mission.mp4

    # Record front camera
    python record_mission.py --connect tcp/192.168.1.10:7447 \
        --camera front --output front_view.mp4

    # Multi-view: 2x2 grid of all cameras
    python record_mission.py --connect tcp/192.168.1.10:7447 \
        --cameras front back down chase --layout grid --output mission_grid.mp4

    # Side-by-side: chase + front
    python record_mission.py --connect tcp/192.168.1.10:7447 \
        --cameras chase front --layout side_by_side --output mission_side.mp4

    # Record with duration limit
    python record_mission.py --connect tcp/192.168.1.10:7447 \
        --camera chase --duration 120 --output mission.mp4

The recording starts immediately upon connection and continues until:
- Ctrl+C is pressed
- Duration limit is reached (if specified)
- The script receives a stop signal
"""

import argparse
import sys
import time
import threading
import subprocess
import tempfile
import os
from pathlib import Path
from typing import Optional, List, Dict, Tuple
from dataclasses import dataclass
from datetime import datetime

import numpy as np
import cv2
import zenoh

from data_types import ImageData


@dataclass
class RecordingConfig:
    """Configuration for recording."""
    cameras: List[str]          # Camera IDs to record
    output: str                 # Output filename
    fps: float = 30.0           # Target FPS
    layout: str = "single"      # single, grid, side_by_side
    duration: Optional[float] = None  # Max duration in seconds
    resolution: Tuple[int, int] = (1280, 720)  # Output resolution


class FrameBuffer:
    """Thread-safe frame buffer for a single camera."""

    def __init__(self, camera_id: str, max_frames: int = 9000):  # 5 min at 30fps
        self.camera_id = camera_id
        self.max_frames = max_frames
        self.frames: List[Tuple[float, np.ndarray]] = []
        self.lock = threading.Lock()
        self.frame_count = 0
        self.last_frame_time = 0.0

    def add_frame(self, timestamp: float, frame: np.ndarray):
        """Add a frame to the buffer."""
        with self.lock:
            if len(self.frames) < self.max_frames:
                self.frames.append((timestamp, frame.copy()))
                self.frame_count += 1
                self.last_frame_time = timestamp

    def get_frames(self) -> List[Tuple[float, np.ndarray]]:
        """Get all frames (clears buffer)."""
        with self.lock:
            frames = self.frames
            self.frames = []
            return frames

    def get_latest(self) -> Optional[np.ndarray]:
        """Get the most recent frame without removing it."""
        with self.lock:
            if self.frames:
                return self.frames[-1][1].copy()
            return None


class MissionRecorder:
    """Records drone camera feeds to video."""

    def __init__(self, connect_endpoint: str, config: RecordingConfig, robot_id: str = "drone"):
        self.connect_endpoint = connect_endpoint
        self.config = config
        self.robot_id = robot_id
        self.running = False
        self.session: Optional[zenoh.Session] = None

        # Frame buffers per camera
        self.buffers: Dict[str, FrameBuffer] = {}
        for cam in config.cameras:
            self.buffers[cam] = FrameBuffer(cam)

        # Recording state
        self.start_time = 0.0
        self.total_frames = 0

    def connect(self) -> bool:
        """Connect to Zenoh and subscribe to camera topics."""
        try:
            print(f"Connecting to Zenoh at {self.connect_endpoint}...")
            zenoh_config = zenoh.Config()
            zenoh_config.insert_json5("connect/endpoints", f'["{self.connect_endpoint}"]')
            self.session = zenoh.open(zenoh_config)
            print("Connected to Zenoh successfully")

            # Subscribe to camera topics
            for camera_id in self.config.cameras:
                # Try both new per-camera topic and legacy topic
                topics = [
                    f"robot/{self.robot_id}/sensor/camera/{camera_id}/rgb",
                ]
                # For front camera, also try legacy single topic
                if camera_id == "front":
                    topics.append(f"robot/{self.robot_id}/sensor/camera/rgb")

                for topic in topics:
                    self.session.declare_subscriber(
                        topic,
                        lambda sample, cid=camera_id: self._on_frame(sample, cid)
                    )
                    print(f"  Subscribed to: {topic}")

            return True

        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def _on_frame(self, sample: zenoh.Sample, camera_id: str):
        """Handle incoming frame."""
        if not self.running:
            return

        try:
            img_data = ImageData.deserialize(bytes(sample.payload))
            frame = img_data.to_numpy()
            timestamp = time.time()

            if camera_id in self.buffers:
                self.buffers[camera_id].add_frame(timestamp, frame)
                self.total_frames += 1

        except Exception:
            pass

    def compose_frame(self, camera_frames: Dict[str, np.ndarray]) -> np.ndarray:
        """Compose frames from multiple cameras into output layout."""
        width, height = self.config.resolution

        if self.config.layout == "single":
            # Single camera - just resize
            cam = self.config.cameras[0]
            if cam in camera_frames and camera_frames[cam] is not None:
                frame = camera_frames[cam]
                return cv2.resize(frame, (width, height))
            else:
                return np.zeros((height, width, 3), dtype=np.uint8)

        elif self.config.layout == "side_by_side":
            # Two cameras side by side
            half_width = width // 2
            output = np.zeros((height, width, 3), dtype=np.uint8)

            for i, cam in enumerate(self.config.cameras[:2]):
                if cam in camera_frames and camera_frames[cam] is not None:
                    frame = cv2.resize(camera_frames[cam], (half_width, height))
                    x_offset = i * half_width
                    output[:, x_offset:x_offset + half_width] = frame

                # Add label
                label = cam.upper()
                cv2.putText(output, label, (i * half_width + 10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            return output

        elif self.config.layout == "grid":
            # 2x2 grid
            half_width = width // 2
            half_height = height // 2
            output = np.zeros((height, width, 3), dtype=np.uint8)

            positions = [(0, 0), (1, 0), (0, 1), (1, 1)]
            for i, cam in enumerate(self.config.cameras[:4]):
                col, row = positions[i]
                x_offset = col * half_width
                y_offset = row * half_height

                if cam in camera_frames and camera_frames[cam] is not None:
                    frame = cv2.resize(camera_frames[cam], (half_width, half_height))
                    output[y_offset:y_offset + half_height,
                           x_offset:x_offset + half_width] = frame

                # Add label
                label = cam.upper()
                cv2.putText(output, label, (x_offset + 10, y_offset + 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            return output

        else:
            raise ValueError(f"Unknown layout: {self.config.layout}")

    def record(self):
        """Start recording."""
        self.running = True
        self.start_time = time.time()

        print("\n" + "=" * 60)
        print("Mission Recorder")
        print("=" * 60)
        print(f"  Cameras: {self.config.cameras}")
        print(f"  Layout: {self.config.layout}")
        print(f"  Output: {self.config.output}")
        print(f"  FPS: {self.config.fps}")
        if self.config.duration:
            print(f"  Max duration: {self.config.duration}s")
        print("=" * 60)
        print("\nRecording... Press Ctrl+C to stop.\n")

        last_status = time.time()

        try:
            while self.running:
                elapsed = time.time() - self.start_time

                # Check duration limit
                if self.config.duration and elapsed >= self.config.duration:
                    print(f"\nDuration limit reached ({self.config.duration}s)")
                    break

                # Status update
                if time.time() - last_status > 2.0:
                    last_status = time.time()
                    frame_counts = {cam: buf.frame_count for cam, buf in self.buffers.items()}
                    print(f"  Recording: {elapsed:.1f}s | Frames: {frame_counts}")

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n\nStopping recording...")

        self.running = False

    def save(self) -> bool:
        """Save recorded frames to video file."""
        print("\nProcessing video...")

        # Collect all frames
        all_frames: Dict[str, List[Tuple[float, np.ndarray]]] = {}
        for cam, buf in self.buffers.items():
            all_frames[cam] = buf.frames
            print(f"  {cam}: {len(buf.frames)} frames")

        if not any(all_frames.values()):
            print("ERROR: No frames captured!")
            return False

        # Find time range
        all_timestamps = []
        for frames in all_frames.values():
            all_timestamps.extend([t for t, _ in frames])

        if not all_timestamps:
            print("ERROR: No timestamps!")
            return False

        start_t = min(all_timestamps)
        end_t = max(all_timestamps)
        duration = end_t - start_t
        print(f"  Duration: {duration:.1f}s")

        # Create video writer
        width, height = self.config.resolution
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(self.config.output, fourcc, self.config.fps, (width, height))

        if not out.isOpened():
            # Try with different codec
            print("  Trying alternative codec...")
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            alt_output = self.config.output.replace('.mp4', '.avi')
            out = cv2.VideoWriter(alt_output, fourcc, self.config.fps, (width, height))
            if not out.isOpened():
                print("ERROR: Could not create video writer!")
                return False
            print(f"  Using AVI format: {alt_output}")

        # Build frame index for each camera (timestamp -> frame)
        frame_indices: Dict[str, Dict[float, np.ndarray]] = {}
        for cam, frames in all_frames.items():
            frame_indices[cam] = {t: f for t, f in frames}

        # Generate output frames at target FPS
        frame_interval = 1.0 / self.config.fps
        current_t = start_t
        frame_num = 0

        while current_t <= end_t:
            # Find nearest frame for each camera
            camera_frames = {}
            for cam, frames in all_frames.items():
                if frames:
                    # Find frame with closest timestamp
                    nearest = min(frames, key=lambda x: abs(x[0] - current_t))
                    # Only use if within reasonable time window
                    if abs(nearest[0] - current_t) < 0.5:
                        camera_frames[cam] = nearest[1]

            # Compose output frame
            output_frame = self.compose_frame(camera_frames)

            # Add timestamp overlay
            elapsed = current_t - start_t
            timestamp_text = f"{elapsed:.1f}s"
            cv2.putText(output_frame, timestamp_text,
                       (width - 100, height - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Write frame
            out.write(output_frame)
            frame_num += 1

            current_t += frame_interval

            # Progress
            if frame_num % 100 == 0:
                progress = (current_t - start_t) / duration * 100
                print(f"  Encoding: {progress:.0f}%", end='\r')

        out.release()
        print(f"\n\nSaved: {self.config.output}")
        print(f"  Frames: {frame_num}")
        print(f"  Duration: {duration:.1f}s")

        # Try to convert to better codec using ffmpeg
        self._convert_with_ffmpeg()

        return True

    def _convert_with_ffmpeg(self):
        """Convert video to H.264 using ffmpeg if available."""
        try:
            # Check if ffmpeg is available
            result = subprocess.run(['ffmpeg', '-version'],
                                   capture_output=True, text=True)
            if result.returncode != 0:
                return

            input_file = self.config.output
            output_file = input_file.replace('.mp4', '_h264.mp4').replace('.avi', '_h264.mp4')

            print(f"\nConverting to H.264 with ffmpeg...")
            result = subprocess.run([
                'ffmpeg', '-y', '-i', input_file,
                '-c:v', 'libx264', '-preset', 'medium', '-crf', '23',
                output_file
            ], capture_output=True, text=True)

            if result.returncode == 0:
                # Replace original with converted
                os.replace(output_file, input_file)
                print(f"  Converted to H.264: {input_file}")
            else:
                print(f"  ffmpeg conversion failed (original file preserved)")

        except FileNotFoundError:
            # ffmpeg not installed
            pass
        except Exception as e:
            print(f"  ffmpeg conversion error: {e}")

    def disconnect(self):
        """Close Zenoh session."""
        if self.session:
            self.session.close()
            self.session = None


def main():
    parser = argparse.ArgumentParser(
        description="Record drone camera feeds to video",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Record chase camera (best for sharing)
  python record_mission.py --connect tcp/192.168.1.10:7447 --camera chase

  # Record multiple cameras in grid layout
  python record_mission.py --connect tcp/192.168.1.10:7447 \\
      --cameras front back down chase --layout grid

  # Record with time limit
  python record_mission.py --connect tcp/192.168.1.10:7447 \\
      --camera chase --duration 60 --output short_clip.mp4
        """
    )
    parser.add_argument("--connect", required=True, help="Zenoh endpoint")
    parser.add_argument("--robot-id", default="drone", help="Robot ID")

    # Camera selection (mutually exclusive for convenience)
    cam_group = parser.add_mutually_exclusive_group()
    cam_group.add_argument("--camera", help="Single camera to record")
    cam_group.add_argument("--cameras", nargs="+",
                          help="Multiple cameras to record")

    parser.add_argument("--layout", choices=["single", "side_by_side", "grid"],
                       default="single", help="Video layout for multiple cameras")
    parser.add_argument("--output", "-o", default=None,
                       help="Output filename (default: mission_TIMESTAMP.mp4)")
    parser.add_argument("--fps", type=float, default=30.0,
                       help="Output FPS (default: 30)")
    parser.add_argument("--resolution", default="1280x720",
                       help="Output resolution WxH (default: 1280x720)")
    parser.add_argument("--duration", type=float, default=None,
                       help="Max recording duration in seconds")

    args = parser.parse_args()

    # Determine cameras to record
    if args.camera:
        cameras = [args.camera]
    elif args.cameras:
        cameras = args.cameras
    else:
        cameras = ["chase"]  # Default

    # Auto-select layout
    layout = args.layout
    if layout == "single" and len(cameras) > 1:
        if len(cameras) == 2:
            layout = "side_by_side"
        else:
            layout = "grid"

    # Parse resolution
    try:
        w, h = args.resolution.lower().split('x')
        resolution = (int(w), int(h))
    except:
        resolution = (1280, 720)

    # Generate output filename
    if args.output:
        output = args.output
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = f"mission_{timestamp}.mp4"

    config = RecordingConfig(
        cameras=cameras,
        output=output,
        fps=args.fps,
        layout=layout,
        duration=args.duration,
        resolution=resolution
    )

    recorder = MissionRecorder(args.connect, config, args.robot_id)

    if not recorder.connect():
        return 1

    try:
        recorder.record()
        recorder.save()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        recorder.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
