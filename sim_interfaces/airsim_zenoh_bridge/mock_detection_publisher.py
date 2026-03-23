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

import zenoh

from data_types import CameraDetectionList, Detection


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
    args = parser.parse_args()

    topic = f"robot/{args.robot_id}/perception/detections/{args.camera}"
    period = 1.0 / max(args.rate, 1.0)

    cfg = zenoh.Config()
    cfg.insert_json5("connect/endpoints", f'["{args.connect}"]')
    session = zenoh.open(cfg)
    pub = session.declare_publisher(topic)
    print(f"Publishing mock detections to {topic} at {args.rate:.1f} Hz")

    frame_id = 0
    started = time.time()

    try:
        while True:
            now = time.time()
            t = now - started

            # Oscillating target around image center.
            bearing_x = 0.4 * math.sin(t * 0.8)
            bearing_y = 0.2 * math.sin(t * 0.5 + 0.6)
            area = int(12000 + 5000 * math.sin(t * 0.6))
            area = max(area, 3000)

            # Optional robustness controls
            if args.bearing_noise > 0:
                bearing_x += random.uniform(-args.bearing_noise, args.bearing_noise)
                bearing_y += random.uniform(-args.bearing_noise, args.bearing_noise)
            bearing_x = max(-1.0, min(1.0, bearing_x))
            bearing_y = max(-1.0, min(1.0, bearing_y))

            if args.area_noise > 0:
                area = int(area * (1.0 + random.uniform(-args.area_noise, args.area_noise)))
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
            if random.random() >= max(0.0, min(1.0, args.drop_rate)):
                pub.put(msg.serialize())

            if frame_id % 20 == 0:
                print(
                    f"frame={frame_id:04d} area={area:5d} "
                    f"bearing=({bearing_x:+.2f}, {bearing_y:+.2f})",
                    end="\r",
                )
            frame_id += 1
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        session.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
