"""
Follow Target Phase

Mission 1 (Solitary Sentinel) baseline phase:
- track a single target class
- maintain a safe stand-off distance using bbox area as a distance proxy
- keep target centered with yaw and vertical corrections
"""

import asyncio
import json
import math
import os
import time

from . import register_phase
from .base import MissionPhase, PhaseResult, PhaseStatus


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@register_phase("follow_target")
class FollowTargetPhase(MissionPhase):
    """
    Baseline single-target follower for Mission 1.

    Config params:
        duration_s: Run duration in seconds (default: 45)
        target_class: Target class name (default from mission config)
        min_confidence: Ignore detections below this confidence (default: 0.25)
        detection_max_age_s: Maximum detection age in seconds (default: 0.5)
        lost_timeout_s: Fail if target is unseen this long (default: 2.0)

        desired_area: Target bbox area proxy for stand-off distance (default: 12000)
        area_tolerance_ratio: Deadband around desired area (default: 0.15)
        forward_gain: Scale area error to forward velocity (default: 2.0)
        max_forward_speed: Maximum forward speed m/s (default: 2.0)
        max_reverse_speed: Maximum reverse speed m/s (default: 1.0)

        yaw_gain: Scale bearing_x to yaw rate (default: 0.8)
        max_yaw_rate: Clamp yaw rate rad/s (default: 0.6)
        vertical_gain: Scale bearing_y to vertical velocity (default: 0.8)
        max_vertical_speed: Clamp vertical velocity m/s (default: 0.8)

        metrics_output_path: Optional JSON output path for run metrics
        min_track_ratio: Minimum tracked frame ratio to pass (default: 0.80)
        max_avg_abs_bearing_x: Max average abs horizontal bearing (default: 0.35)
        max_avg_abs_area_error_ratio: Max average abs area error ratio (default: 0.55)
        max_avg_command_delta: Max average command delta for smoothness (default: 1.0)
    """

    async def execute(self) -> PhaseResult:
        target_class = self.get_param("target_class", "orange ball")
        duration_s = float(self.config.get("duration_s", 45.0))
        min_conf = float(self.config.get("min_confidence", 0.25))
        detection_max_age_s = float(self.config.get("detection_max_age_s", 0.5))
        lost_timeout_s = float(self.config.get("lost_timeout_s", 2.0))

        desired_area = float(self.config.get("desired_area", 12000.0))
        area_tolerance = float(self.config.get("area_tolerance_ratio", 0.15))
        forward_gain = float(self.config.get("forward_gain", 2.0))
        max_forward = float(self.config.get("max_forward_speed", 2.0))
        max_reverse = float(self.config.get("max_reverse_speed", 1.0))

        yaw_gain = float(self.config.get("yaw_gain", 0.8))
        max_yaw_rate = float(self.config.get("max_yaw_rate", 0.6))
        vertical_gain = float(self.config.get("vertical_gain", 0.8))
        max_vertical = float(self.config.get("max_vertical_speed", 0.8))

        metrics_output_path = self.config.get("metrics_output_path", "")
        if not metrics_output_path:
            output_dir = self.get_param("output_dir", "outputs")
            mission_name = self.get_param("mission_name", "mission")
            metrics_output_path = os.path.join(output_dir, f"{mission_name}_metrics.json")

        min_track_ratio = float(self.config.get("min_track_ratio", 0.80))
        max_avg_abs_bearing_x = float(self.config.get("max_avg_abs_bearing_x", 0.35))
        max_avg_abs_area_error_ratio = float(self.config.get("max_avg_abs_area_error_ratio", 0.55))
        max_avg_command_delta = float(self.config.get("max_avg_command_delta", 1.0))

        if desired_area <= 0.0:
            return PhaseResult(
                status=PhaseStatus.FAILED,
                message=f"Invalid desired_area={desired_area}. It must be > 0.",
            )

        print(f"  Target class: '{target_class}'")
        print(f"  Duration: {duration_s:.1f}s")
        print(f"  Desired bbox area: {desired_area:.0f}px")

        start = time.time()
        last_seen = start

        metrics = {
            "target_class": target_class,
            "duration_s": duration_s,
            "frames_tracked": 0,
            "frames_no_target": 0,
            "max_lost_streak_s": 0.0,
            "avg_abs_bearing_x": 0.0,
            "avg_abs_bearing_y": 0.0,
            "avg_abs_area_error_ratio": 0.0,
            "avg_command_delta": 0.0,
            "track_ratio": 0.0,
            "acceptance": {},
        }
        sum_abs_bx = 0.0
        sum_abs_by = 0.0
        sum_abs_area_err = 0.0
        sum_cmd_delta = 0.0
        command_delta_samples = 0
        prev_cmd = None

        def finalize_metrics(current_elapsed: float) -> None:
            total_frames = metrics["frames_tracked"] + metrics["frames_no_target"]
            tracked = max(metrics["frames_tracked"], 1)
            metrics["duration_s"] = current_elapsed
            metrics["avg_abs_bearing_x"] = sum_abs_bx / tracked
            metrics["avg_abs_bearing_y"] = sum_abs_by / tracked
            metrics["avg_abs_area_error_ratio"] = sum_abs_area_err / tracked
            metrics["avg_command_delta"] = (sum_cmd_delta / command_delta_samples) if command_delta_samples > 0 else 0.0
            metrics["track_ratio"] = (
                metrics["frames_tracked"] / total_frames if total_frames > 0 else 0.0
            )
            metrics["acceptance"] = {
                "min_track_ratio": min_track_ratio,
                "max_avg_abs_bearing_x": max_avg_abs_bearing_x,
                "max_avg_abs_area_error_ratio": max_avg_abs_area_error_ratio,
                "max_avg_command_delta": max_avg_command_delta,
            }

        def export_metrics() -> None:
            if not metrics_output_path:
                return
            out_path = os.path.abspath(metrics_output_path)
            out_dir = os.path.dirname(out_path)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(metrics, f, indent=2)
            print(f"\n  Metrics written to: {out_path}")

        def acceptance_passed() -> bool:
            return (
                metrics["track_ratio"] >= min_track_ratio
                and metrics["avg_abs_bearing_x"] <= max_avg_abs_bearing_x
                and metrics["avg_abs_area_error_ratio"] <= max_avg_abs_area_error_ratio
                and metrics["avg_command_delta"] <= max_avg_command_delta
            )

        while self.state.running and not self.is_cancelled:
            elapsed = time.time() - start
            if elapsed >= duration_s:
                self.drone.hover()
                finalize_metrics(elapsed)
                self.state.phase_data["follow_target_metrics"] = metrics
                export_metrics()
                if acceptance_passed():
                    return PhaseResult(
                        status=PhaseStatus.COMPLETED,
                        message=f"Tracked '{target_class}' for {elapsed:.1f}s (acceptance passed)",
                        data=metrics,
                    )
                return PhaseResult(
                    status=PhaseStatus.FAILED,
                    message=f"Acceptance failed after {elapsed:.1f}s",
                    data=metrics,
                )

            cam_id, det = self.detection.get_best_detection(target_class, max_age=detection_max_age_s)
            if det is None or det.confidence < min_conf:
                self.drone.hover()
                metrics["frames_no_target"] += 1
                lost_for = time.time() - last_seen
                metrics["max_lost_streak_s"] = max(metrics["max_lost_streak_s"], lost_for)

                if lost_for > lost_timeout_s:
                    finalize_metrics(elapsed)
                    self.state.phase_data["follow_target_metrics"] = metrics
                    export_metrics()
                    return PhaseResult(
                        status=PhaseStatus.FAILED,
                        message=f"Lost target for {lost_for:.1f}s (> {lost_timeout_s:.1f}s)",
                        data=metrics,
                    )
                await asyncio.sleep(0.1)
                continue

            last_seen = time.time()
            metrics["frames_tracked"] += 1

            bearing_x = float(det.bearing_x)
            bearing_y = float(det.bearing_y)
            area = max(float(det.area), 1.0)

            area_error_ratio = (desired_area - area) / desired_area
            if abs(area_error_ratio) < area_tolerance:
                area_error_ratio = 0.0

            forward_speed = _clamp(forward_gain * area_error_ratio, -max_reverse, max_forward)
            yaw_rate = _clamp(yaw_gain * bearing_x, -max_yaw_rate, max_yaw_rate)
            vz = _clamp(vertical_gain * bearing_y, -max_vertical, max_vertical)

            # DroneService expects world-frame NED velocities.
            _, _, _, yaw = self.drone.pose
            vx = forward_speed * math.cos(yaw)
            vy = forward_speed * math.sin(yaw)
            self.drone.send_velocity(vx, vy, vz, yaw_rate)

            sum_abs_bx += abs(bearing_x)
            sum_abs_by += abs(bearing_y)
            sum_abs_area_err += abs(area_error_ratio)
            cmd = (vx, vy, vz, yaw_rate)
            if prev_cmd is not None:
                delta = (
                    abs(cmd[0] - prev_cmd[0])
                    + abs(cmd[1] - prev_cmd[1])
                    + abs(cmd[2] - prev_cmd[2])
                    + abs(cmd[3] - prev_cmd[3])
                )
                sum_cmd_delta += delta
                command_delta_samples += 1
            prev_cmd = cmd

            print(
                f"  [{cam_id}] t={elapsed:5.1f}s conf={det.confidence:.2f} "
                f"area={area:7.0f} vx={vx:+.2f} vz={vz:+.2f} yaw={yaw_rate:+.2f}",
                end="\r",
            )
            await asyncio.sleep(0.1)

        self.drone.hover()
        finalize_metrics(time.time() - start)
        self.state.phase_data["follow_target_metrics"] = metrics
        export_metrics()
        return PhaseResult(
            status=PhaseStatus.FAILED,
            message="Cancelled or stopped",
            data=metrics,
        )
