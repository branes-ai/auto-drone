#!/usr/bin/env python3
"""
Mock detection publisher for local mission-controller testing.

Publishes a synthetic single target on:
  robot/{id}/perception/detections/{camera}
"""

import argparse
import math
import random
import sys
import time
from collections import deque

import zenoh

from data_types import CameraDetectionList, Detection

NOISE_LEVELS = {
    "0": 0.0,
    "low": 0.05,
    "medium": 0.10,
    "high": 0.20,
    "very_high": 0.40,
}

TRAJECTORIES = ("oscillate", "straight", "curve", "speed_ramp")


def _build_target_state(trajectory: str, t: float) -> tuple[float, float, int]:
    """Generate synthetic target bearing/area for the chosen trajectory."""
    if trajectory == "straight":
        phase = (t * 0.12) % 2.0
        sweep = phase if phase <= 1.0 else 2.0 - phase
        bearing_x = -0.55 + 1.10 * sweep
        bearing_y = 0.06 * math.sin(t * 0.35)
        area = int(11000 + 1800 * math.sin(t * 0.25))
    elif trajectory == "curve":
        bearing_x = 0.45 * math.sin(t * 0.45)
        bearing_y = 0.18 * math.sin(t * 0.25 + 1.1)
        area = int(12000 + 3200 * math.sin(t * 0.40 + 0.7))
    elif trajectory == "speed_ramp":
        speed_scale = 0.45 + 0.30 * (1.0 + math.sin(t * 0.18))
        bearing_x = 0.50 * math.sin(t * speed_scale)
        bearing_y = 0.15 * math.sin(t * 0.42 + 0.4)
        area = int(12000 + 4200 * math.sin(t * (0.35 + 0.10 * math.sin(t * 0.20))))
    else:
        bearing_x = 0.4 * math.sin(t * 0.8)
        bearing_y = 0.2 * math.sin(t * 0.5 + 0.6)
        area = int(12000 + 5000 * math.sin(t * 0.6))

    return bearing_x, bearing_y, max(area, 3000)


def main() -> int:
    parser = argparse.ArgumentParser(description="Mock detection publisher")
    parser.add_argument("--connect", type=str, default="tcp/127.0.0.1:7447", help="Zenoh endpoint")
    parser.add_argument("--robot-id", type=str, default="drone", help="Robot ID")
    parser.add_argument("--camera", type=str, default="front", help="Camera topic suffix")
    parser.add_argument("--target-class", type=str, default="orange ball", help="Detection class name")
    parser.add_argument("--rate", type=float, default=10.0, help="Publish rate in Hz")
    parser.add_argument("--drop-rate", type=float, default=0.0, help="Probability [0..1] of skipping a frame")
    parser.add_argument("--bearing-noise", type=float, default=0.0, help="Uniform noise amplitude added to bearings")
    parser.add_argument("--area-noise", type=float, default=0.0, help="Uniform relative noise for area, e.g. 0.2")
    parser.add_argument("--sensor-noise-level", choices=list(NOISE_LEVELS.keys()), default=None,
                        help="Named sensor noise profile applied to both bearing and area noise")
    parser.add_argument("--trajectory", choices=TRAJECTORIES, default="oscillate",
                        help="Target motion profile")
    parser.add_argument("--latency-ms", type=float, default=0.0,
                        help="Fixed publish latency in milliseconds")
    parser.add_argument("--dropout-burst-prob", type=float, default=0.0,
                        help="Probability [0..1] of starting a multi-frame dropout burst")
    parser.add_argument("--dropout-burst-min-s", type=float, default=0.5,
                        help="Minimum dropout burst duration in seconds")
    parser.add_argument("--dropout-burst-max-s", type=float, default=2.0,
                        help="Maximum dropout burst duration in seconds")
    parser.add_argument("--seed", type=int, default=None, help="Optional random seed for reproducible runs")
    args = parser.parse_args()

    if args.sensor_noise_level is not None:
        level = NOISE_LEVELS[args.sensor_noise_level]
        args.bearing_noise = level
        args.area_noise = level

    rng = random.Random(args.seed)

    topic = f"robot/{args.robot_id}/perception/detections/{args.camera}"
    period = 1.0 / max(args.rate, 1.0)

    cfg = zenoh.Config()
    cfg.insert_json5("connect/endpoints", f'["{args.connect}"]')
    session = zenoh.open(cfg)
    pub = session.declare_publisher(topic)
    print(f"Publishing mock detections to {topic} at {args.rate:.1f} Hz")

    frame_id = 0
    started = time.time()
    latency_s = max(args.latency_ms, 0.0) / 1000.0
    pending_msgs = deque()
    dropout_until = 0.0

    try:
        while True:
            now = time.time()
            t = now - started

            while pending_msgs and pending_msgs[0][0] <= now:
                _, payload = pending_msgs.popleft()
                pub.put(payload)

            bearing_x, bearing_y, area = _build_target_state(args.trajectory, t)

            if args.bearing_noise > 0:
                bearing_x += rng.uniform(-args.bearing_noise, args.bearing_noise)
                bearing_y += rng.uniform(-args.bearing_noise, args.bearing_noise)
            bearing_x = max(-1.0, min(1.0, bearing_x))
            bearing_y = max(-1.0, min(1.0, bearing_y))

            if args.area_noise > 0:
                area = int(area * (1.0 + rng.uniform(-args.area_noise, args.area_noise)))
                area = max(area, 1000)

            width = 640
            height = 480
            center_x = int(width * (0.5 + 0.5 * bearing_x))
            center_y = int(height * (0.5 + 0.5 * bearing_y))
            bbox_w = int(math.sqrt(area))
            bbox_h = int(math.sqrt(area))
            bbox_x = max(center_x - bbox_w // 2, 0)
            bbox_y = max(center_y - bbox_h // 2, 0)

            det = Detection(
                class_id=0,
                class_name=args.target_class,
                confidence=0.92,
                bbox_x=bbox_x,
                bbox_y=bbox_y,
                bbox_w=bbox_w,
                bbox_h=bbox_h,
                center_x=center_x,
                center_y=center_y,
                area=area,
                bearing_x=bearing_x,
                bearing_y=bearing_y,
            )
            msg = CameraDetectionList(
                timestamp_us=int(now * 1_000_000),
                frame_id=frame_id,
                image_width=width,
                image_height=height,
                inference_time_ms=2.0,
                camera_id=args.camera,
                camera_position=(0.3, 0.0, -0.05),
                camera_orientation=(0.0, -15.0, 0.0),
                detections=[det],
            )

            should_publish = True
            if now < dropout_until:
                should_publish = False
            elif args.dropout_burst_prob > 0 and rng.random() < max(0.0, min(1.0, args.dropout_burst_prob)):
                burst_s = rng.uniform(
                    max(args.dropout_burst_min_s, 0.0),
                    max(args.dropout_burst_max_s, max(args.dropout_burst_min_s, 0.0)),
                )
                dropout_until = now + burst_s
                should_publish = False

            if should_publish and rng.random() >= max(0.0, min(1.0, args.drop_rate)):
                payload = msg.serialize()
                if latency_s > 0:
                    pending_msgs.append((now + latency_s, payload))
                else:
                    pub.put(payload)

            if frame_id % 20 == 0:
                print(
                    f"frame={frame_id:04d} traj={args.trajectory:>10} area={area:5d} "
                    f"bearing=({bearing_x:+.2f}, {bearing_y:+.2f})",
                    end="\r",
                )
            frame_id += 1
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        while pending_msgs:
            _, payload = pending_msgs.popleft()
            pub.put(payload)
        session.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
