#!/usr/bin/env python3
"""Mission 1 robustness experiment runner for mock perception/control noise."""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from statistics import mean

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "packages"))
sys.path.insert(0, str(ROOT / "sim_interfaces" / "airsim_zenoh_bridge"))

from mission_framework.config import MissionConfig  # noqa: E402

NOISE_LEVELS = {
    "0": 0.0,
    "low": 0.05,
    "medium": 0.10,
    "high": 0.20,
    "very_high": 0.40,
}

DEFAULT_START_POSES = [
    {"x": 0.0, "y": 0.0, "z": -5.0, "yaw_deg": 0.0},
    {"x": 2.5, "y": -1.5, "z": -5.5, "yaw_deg": 20.0},
    {"x": -3.0, "y": 1.0, "z": -4.5, "yaw_deg": -25.0},
]

TRAJECTORIES = ["oscillate", "straight", "curve", "speed_ramp"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Mission 1 robustness experiments")
    default_config = ROOT / "packages" / "mission_framework" / "configs" / "solitary_sentinel_tuned.yaml"
    default_mission_timeout = MissionConfig.from_yaml(default_config).mission_timeout
    parser.add_argument(
        "--config",
        default=str(default_config),
        help="Mission config YAML to use as the experiment baseline",
    )
    parser.add_argument("--connect", default="tcp/127.0.0.1:7447", help="Zenoh endpoint")
    parser.add_argument(
        "--listen",
        default="tcp/0.0.0.0:7447",
        help="Zenoh listen endpoint for the mock bridge",
    )
    parser.add_argument("--python", default=sys.executable, help="Python executable for child processes")
    parser.add_argument("--repeats", type=int, default=3, help="Runs per noise pair")
    parser.add_argument(
        "--sensor-levels",
        default=",".join(NOISE_LEVELS.keys()),
        help="Comma-separated sensor noise levels to evaluate",
    )
    parser.add_argument(
        "--actuator-levels",
        default=",".join(NOISE_LEVELS.keys()),
        help="Comma-separated actuator noise levels to evaluate",
    )
    parser.add_argument(
        "--trajectories",
        default=",".join(TRAJECTORIES),
        help="Comma-separated trajectory profiles to evaluate",
    )
    parser.add_argument("--latency-ms", type=float, default=150.0, help="Fixed detection latency in milliseconds")
    parser.add_argument("--dropout-burst-prob", type=float, default=0.02, help="Probability of starting a detection dropout burst")
    parser.add_argument("--dropout-burst-min-s", type=float, default=0.5, help="Minimum dropout burst duration in seconds")
    parser.add_argument("--dropout-burst-max-s", type=float, default=1.5, help="Maximum dropout burst duration in seconds")
    parser.add_argument(
        "--output-dir",
        default=str(ROOT / "outputs" / "mission1_robustness"),
        help="Output directory for experiment artifacts",
    )
    parser.add_argument("--bridge-start-delay", type=float, default=3.0, help="Seconds to wait after bridge startup")
    parser.add_argument("--detector-start-delay", type=float, default=2.0, help="Seconds to wait after detector startup")
    parser.add_argument(
        "--mission-timeout",
        type=float,
        default=default_mission_timeout,
        help="Timeout for a single mission run in seconds",
    )
    return parser.parse_args()


def acceptance_pass(metrics: dict) -> bool:
    acceptance = metrics["acceptance"]
    return (
        metrics["track_ratio"] >= acceptance["min_track_ratio"]
        and metrics["avg_abs_bearing_x"] <= acceptance["max_avg_abs_bearing_x"]
        and metrics["avg_abs_area_error_ratio"] <= acceptance["max_avg_abs_area_error_ratio"]
        and metrics["avg_command_delta"] <= acceptance["max_avg_command_delta"]
    )


def metrics_output_path(config_path: Path) -> Path:
    config = MissionConfig.from_yaml(config_path)
    return ROOT / config.output_dir / f"{config.name}_metrics.json"


def load_runtime_params(config_path: Path) -> dict:
    """Load mission-level runtime parameters needed by the mock helpers."""
    config = MissionConfig.from_yaml(config_path)
    camera_id = config.cameras[0] if config.cameras else "front"
    return {
        "robot_id": config.robot_id,
        "camera_id": camera_id,
        "target_class": config.target_class,
        "metrics_path": ROOT / config.output_dir / f"{config.name}_metrics.json",
    }


def start_process(cmd: list[str], log_path: Path, env: dict[str, str], cwd: Path) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_handle = open(log_path, "w", encoding="utf-8")
    proc = subprocess.Popen(
        cmd,
        cwd=cwd,
        env=env,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0,
    )
    proc._log_handle = log_handle  # type: ignore[attr-defined]
    return proc


def stop_process(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    if proc.poll() is None:
        try:
            if os.name == "nt":
                proc.send_signal(signal.CTRL_BREAK_EVENT)
                time.sleep(1.0)
            proc.terminate()
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                proc.kill()
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
            except ProcessLookupError:
                pass
        except ProcessLookupError:
            pass
    log_handle = getattr(proc, "_log_handle", None)
    if log_handle is not None:
        log_handle.close()


def run_one(
    python_exe: str,
    connect: str,
    config_path: Path,
    output_dir: Path,
    sensor_level: str,
    actuator_level: str,
    repeat_idx: int,
    pose: dict,
    trajectory: str,
    latency_ms: float,
    dropout_burst_prob: float,
    dropout_burst_min_s: float,
    dropout_burst_max_s: float,
    bridge_delay: float,
    detector_delay: float,
    mission_timeout: float,
    listen: str,
    robot_id: str,
    camera_id: str,
    target_class: str,
    metrics_path: Path,
) -> dict:
    env = os.environ.copy()
    pythonpath_parts = [
        str(ROOT / "packages"),
        str(ROOT / "sim_interfaces" / "airsim_zenoh_bridge"),
    ]
    existing_pythonpath = env.get("PYTHONPATH")
    if existing_pythonpath:
        pythonpath_parts.append(existing_pythonpath)
    env["PYTHONPATH"] = os.pathsep.join(pythonpath_parts)

    run_name = (
        f"trajectory_{trajectory}__sensor_{sensor_level}__actuator_{actuator_level}__run_{repeat_idx + 1}"
    )
    run_dir = output_dir / run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    if metrics_path.exists():
        metrics_path.unlink()

    bridge_cmd = [
        python_exe,
        "-u",
        "sim_interfaces/airsim_zenoh_bridge/mock_bridge.py",
        "--zenoh-listen",
        listen,
        "--robot-id",
        robot_id,
        "--actuator-noise-level",
        actuator_level,
        "--start-x",
        str(pose["x"]),
        "--start-y",
        str(pose["y"]),
        "--start-z",
        str(pose["z"]),
        "--start-yaw-deg",
        str(pose["yaw_deg"]),
        "--seed",
        str(10_000 + repeat_idx),
    ]
    detector_cmd = [
        python_exe,
        "-u",
        "sim_interfaces/airsim_zenoh_bridge/mock_detection_publisher.py",
        "--connect",
        connect,
        "--robot-id",
        robot_id,
        "--camera",
        camera_id,
        "--target-class",
        target_class,
        "--sensor-noise-level",
        sensor_level,
        "--trajectory",
        trajectory,
        "--latency-ms",
        str(latency_ms),
        "--dropout-burst-prob",
        str(dropout_burst_prob),
        "--dropout-burst-min-s",
        str(dropout_burst_min_s),
        "--dropout-burst-max-s",
        str(dropout_burst_max_s),
        "--seed",
        str(20_000 + repeat_idx),
    ]
    mission_cmd = [
        python_exe,
        "sim_interfaces/airsim_zenoh_bridge/run_mission.py",
        "--connect",
        connect,
        "--config",
        str(config_path),
    ]

    bridge_proc = detector_proc = None
    mission_log = run_dir / "mission.log"
    mission_returncode = None
    mission_timed_out = False
    started_at = time.time()
    try:
        bridge_proc = start_process(bridge_cmd, run_dir / "bridge.log", env, ROOT)
        time.sleep(bridge_delay)
        detector_proc = start_process(detector_cmd, run_dir / "detector.log", env, ROOT)
        time.sleep(detector_delay)

        with open(mission_log, "w", encoding="utf-8") as log_handle:
            try:
                mission_proc = subprocess.run(
                    mission_cmd,
                    cwd=ROOT,
                    env=env,
                    stdout=log_handle,
                    stderr=subprocess.STDOUT,
                    timeout=mission_timeout,
                )
                mission_returncode = mission_proc.returncode
            except subprocess.TimeoutExpired:
                mission_timed_out = True
                mission_returncode = -1
                log_handle.write(f"\nMission timed out after {mission_timeout:.1f}s\n")
    finally:
        stop_process(detector_proc)
        stop_process(bridge_proc)

    elapsed = time.time() - started_at

    if not metrics_path.exists():
        return {
            "trajectory": trajectory,
            "sensor_noise": sensor_level,
            "actuator_noise": actuator_level,
            "repeat": repeat_idx + 1,
            "start_pose": pose,
            "mission_returncode": mission_returncode,
            "mission_timed_out": mission_timed_out,
            "elapsed_s": elapsed,
            "metrics_found": False,
            "pass": False,
        }

    run_metrics_path = run_dir / "metrics.json"
    shutil.copy2(metrics_path, run_metrics_path)
    try:
        with open(run_metrics_path, "r", encoding="utf-8") as f:
            metrics = json.load(f)
        passed = acceptance_pass(metrics) and mission_returncode == 0
    except (OSError, json.JSONDecodeError, KeyError, TypeError):
        return {
            "trajectory": trajectory,
            "sensor_noise": sensor_level,
            "actuator_noise": actuator_level,
            "repeat": repeat_idx + 1,
            "start_pose": pose,
            "mission_returncode": mission_returncode,
            "mission_timed_out": mission_timed_out,
            "elapsed_s": elapsed,
            "metrics_found": False,
            "pass": False,
        }

    return {
        "trajectory": trajectory,
        "sensor_noise": sensor_level,
        "actuator_noise": actuator_level,
        "repeat": repeat_idx + 1,
        "start_pose": pose,
        "mission_returncode": mission_returncode,
        "mission_timed_out": mission_timed_out,
        "elapsed_s": elapsed,
        "metrics_found": True,
        "pass": passed,
        "metrics": metrics,
    }


def aggregate_results(results: list[dict]) -> list[dict]:
    grouped: dict[tuple[str, str, str], list[dict]] = {}
    for item in results:
        grouped.setdefault((item["trajectory"], item["sensor_noise"], item["actuator_noise"]), []).append(item)

    aggregates = []
    for (trajectory, sensor_level, actuator_level), runs in grouped.items():
        metric_runs = [r["metrics"] for r in runs if r.get("metrics_found")]
        pass_count = sum(1 for r in runs if r["pass"])
        aggregate = {
            "trajectory": trajectory,
            "sensor_noise": sensor_level,
            "actuator_noise": actuator_level,
            "runs": len(runs),
            "pass_count": pass_count,
            "pass_rate": pass_count / len(runs) if runs else 0.0,
            "avg_track_ratio": mean(m["track_ratio"] for m in metric_runs) if metric_runs else 0.0,
            "avg_abs_bearing_x": mean(m["avg_abs_bearing_x"] for m in metric_runs) if metric_runs else 0.0,
            "avg_abs_area_error_ratio": mean(m["avg_abs_area_error_ratio"] for m in metric_runs) if metric_runs else 0.0,
            "avg_command_delta": mean(m["avg_command_delta"] for m in metric_runs) if metric_runs else 0.0,
        }
        aggregates.append(aggregate)
    return sorted(
        aggregates,
        key=lambda x: (
            TRAJECTORIES.index(x["trajectory"]),
            list(NOISE_LEVELS).index(x["sensor_noise"]),
            list(NOISE_LEVELS).index(x["actuator_noise"]),
        ),
    )


def stable_summary(aggregates: list[dict]) -> dict:
    fully_stable = [a for a in aggregates if a["pass_rate"] == 1.0]
    failures = [a for a in aggregates if a["pass_rate"] < 1.0]

    def noise_rank(name: str) -> float:
        return NOISE_LEVELS[name]

    best_fully_stable = None
    if fully_stable:
        best_fully_stable = max(
            fully_stable,
            key=lambda a: (
                noise_rank(a["sensor_noise"]),
                noise_rank(a["actuator_noise"]),
                a["pass_rate"],
            ),
        )

    return {
        "fully_stable_pairs": len(fully_stable),
        "best_fully_stable_pair": {
            "trajectory": best_fully_stable["trajectory"],
            "sensor_noise": best_fully_stable["sensor_noise"],
            "actuator_noise": best_fully_stable["actuator_noise"],
        } if best_fully_stable else None,
        "failure_conditions": [
            {
                "trajectory": a["trajectory"],
                "sensor_noise": a["sensor_noise"],
                "actuator_noise": a["actuator_noise"],
                "pass_rate": a["pass_rate"],
            }
            for a in failures
        ],
    }


def write_markdown_report(output_dir: Path, summary_json: dict) -> Path:
    summary = summary_json["summary"]
    aggregates = summary_json["aggregates"]
    failures = summary["failure_conditions"]
    stable_pairs = [a for a in aggregates if a["pass_rate"] == 1.0]

    report_lines = [
        "# Mission 1 Robustness Report",
        "",
        f"Config: `{summary_json['config']}`",
        "",
        "## Scenario Setup",
        "",
        f"- Trajectories: {', '.join(summary_json['trajectories'])}",
        f"- Sensor noise levels: {', '.join(summary_json['sensor_noise_levels'])}",
        f"- Actuator noise levels: {', '.join(summary_json['actuator_noise_levels'])}",
        f"- Repeats per noise pair: {summary_json['repeats']}",
        f"- Detection latency: {summary_json['latency_ms']} ms",
        f"- Dropout burst probability: {summary_json['dropout_burst_prob']}",
        f"- Dropout burst window: {summary_json['dropout_burst_min_s']}-{summary_json['dropout_burst_max_s']} s",
        "",
        "## Top-Level Result",
        "",
        f"- Fully stable trajectory/noise pairs: {summary['fully_stable_pairs']}",
        "",
        "## Stable Conditions",
        "",
    ]

    best_pair = summary.get("best_fully_stable_pair")
    if best_pair is not None:
        report_lines[report_lines.index("## Stable Conditions") - 1:report_lines.index("## Stable Conditions") - 1] = [
            f"- Best fully stable pair: trajectory=`{best_pair['trajectory']}`, sensor=`{best_pair['sensor_noise']}`, actuator=`{best_pair['actuator_noise']}`",
            "",
        ]
    else:
        report_lines[report_lines.index("## Stable Conditions") - 1:report_lines.index("## Stable Conditions") - 1] = [
            "- No fully stable pair was observed in this sweep.",
            "",
        ]

    for item in stable_pairs[:15]:
        report_lines.append(
            f"- `{item['trajectory']}` | sensor=`{item['sensor_noise']}` | actuator=`{item['actuator_noise']}` | pass_rate={item['pass_rate']:.2f}"
        )

    report_lines.extend(["", "## Failure Conditions", ""])
    if failures:
        for item in failures:
            report_lines.append(
                f"- `{item['trajectory']}` | sensor=`{item['sensor_noise']}` | actuator=`{item['actuator_noise']}` | pass_rate={item['pass_rate']:.2f}"
            )
    else:
        report_lines.append("- No failure conditions observed.")

    report_lines.extend([
        "",
        "## Notes",
        "",
        "- Temporal dropout bursts and fixed detection latency were enabled to make the mock perception path more realistic.",
        "- Multiple target trajectories were exercised instead of a single oscillating path.",
        "- Use the JSON/CSV outputs for deeper analysis and the per-run folders for raw logs.",
        "",
    ])

    report_path = output_dir / "summary_report.md"
    report_path.write_text("\n".join(report_lines), encoding="utf-8")
    return report_path


def main() -> int:
    args = parse_args()
    config_path = Path(args.config).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    runtime_params = load_runtime_params(config_path)

    sensor_levels = [x.strip() for x in args.sensor_levels.split(",") if x.strip()]
    actuator_levels = [x.strip() for x in args.actuator_levels.split(",") if x.strip()]
    trajectories = [x.strip() for x in args.trajectories.split(",") if x.strip()]

    if args.repeats <= 0:
        raise SystemExit("--repeats must be greater than 0")
    if not sensor_levels:
        raise SystemExit("At least one sensor noise level must be provided")
    if not actuator_levels:
        raise SystemExit("At least one actuator noise level must be provided")
    if not trajectories:
        raise SystemExit("At least one trajectory must be provided")

    for level in sensor_levels + actuator_levels:
        if level not in NOISE_LEVELS:
            raise SystemExit(f"Unknown noise level: {level}")
    for trajectory in trajectories:
        if trajectory not in TRAJECTORIES:
            raise SystemExit(f"Unknown trajectory: {trajectory}")

    results = []
    for trajectory in trajectories:
        for sensor_level in sensor_levels:
            for actuator_level in actuator_levels:
                for repeat_idx in range(args.repeats):
                    pose = DEFAULT_START_POSES[repeat_idx % len(DEFAULT_START_POSES)]
                    print(
                        f"Running trajectory={trajectory} sensor={sensor_level} actuator={actuator_level} "
                        f"repeat={repeat_idx + 1}/{args.repeats} start=({pose['x']}, {pose['y']}, {pose['z']}, {pose['yaw_deg']})"
                    )
                    result = run_one(
                        python_exe=args.python,
                        connect=args.connect,
                        config_path=config_path,
                        output_dir=output_dir,
                        sensor_level=sensor_level,
                        actuator_level=actuator_level,
                        repeat_idx=repeat_idx,
                        pose=pose,
                        trajectory=trajectory,
                        latency_ms=args.latency_ms,
                        dropout_burst_prob=args.dropout_burst_prob,
                        dropout_burst_min_s=args.dropout_burst_min_s,
                        dropout_burst_max_s=args.dropout_burst_max_s,
                        bridge_delay=args.bridge_start_delay,
                        detector_delay=args.detector_start_delay,
                        mission_timeout=args.mission_timeout,
                        listen=args.listen,
                        robot_id=runtime_params["robot_id"],
                        camera_id=runtime_params["camera_id"],
                        target_class=runtime_params["target_class"],
                        metrics_path=runtime_params["metrics_path"],
                    )
                    results.append(result)
                    status = "PASS" if result["pass"] else "FAIL"
                    print(f"  -> {status}")

    aggregates = aggregate_results(results)
    summary = stable_summary(aggregates)

    summary_json = {
        "config": str(config_path),
        "trajectories": trajectories,
        "sensor_noise_levels": sensor_levels,
        "actuator_noise_levels": actuator_levels,
        "repeats": args.repeats,
        "start_poses": DEFAULT_START_POSES,
        "latency_ms": args.latency_ms,
        "dropout_burst_prob": args.dropout_burst_prob,
        "dropout_burst_min_s": args.dropout_burst_min_s,
        "dropout_burst_max_s": args.dropout_burst_max_s,
        "runs": results,
        "aggregates": aggregates,
        "summary": summary,
    }

    json_path = output_dir / "summary.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(summary_json, f, indent=2)

    csv_path = output_dir / "summary.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "trajectory",
                "sensor_noise",
                "actuator_noise",
                "runs",
                "pass_count",
                "pass_rate",
                "avg_track_ratio",
                "avg_abs_bearing_x",
                "avg_abs_area_error_ratio",
                "avg_command_delta",
            ],
        )
        writer.writeheader()
        writer.writerows(aggregates)

    report_path = write_markdown_report(output_dir, summary_json)

    print()
    print(f"Summary JSON:   {json_path}")
    print(f"Summary CSV:    {csv_path}")
    print(f"Summary report: {report_path}")
    best_pair = summary["best_fully_stable_pair"]
    if best_pair is not None:
        print(
            "Best fully stable pair: "
            f"trajectory={best_pair['trajectory']} "
            f"sensor={best_pair['sensor_noise']} "
            f"actuator={best_pair['actuator_noise']}"
        )
    else:
        print("No fully stable pair was observed in the evaluated grid.")
    if summary["failure_conditions"]:
        print("Fails under:")
        for item in summary["failure_conditions"][:20]:
            print(
                f"  trajectory={item['trajectory']} sensor={item['sensor_noise']} "
                f"actuator={item['actuator_noise']} pass_rate={item['pass_rate']:.2f}"
            )
    else:
        print("No failure conditions observed in the evaluated grid.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
