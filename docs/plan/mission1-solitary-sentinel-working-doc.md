# Mission 1: Solitary Sentinel - Why, What, How

This note captures the purpose, implementation scope, execution flow, and
learnings from the Mission 1 (`Solitary Sentinel`) mock-validation work.

It is intended to accompany the Mission 1 validation PR and preserve the
reasoning behind the baseline, the validation method, and the main outcomes.

## Why

Mission 1 is the baseline capability for the mission framework. It answers a
simple but important question:

- Can the drone reliably follow a single, visually distinct target?

This is the first capability in the mission progression because it validates the
core loop before more complex missions are attempted:

- perception input arrives
- mission logic selects the target behavior
- the controller generates drone commands
- metrics confirm whether the behavior is acceptable

The mock pipeline was used first because it provides a fast, repeatable way to
validate the mission flow, acceptance criteria, and robustness assumptions
before moving to heavier simulator-backed testing.

## What

The Mission 1 work covered four practical areas.

### 1. Baseline Mission Validation

The existing `FollowTargetPhase` and `solitary_sentinel.yaml` configuration were
run end-to-end using the mock bridge and mock detection publisher.

Acceptance criteria checked:

- `track_ratio >= 0.80`
- `avg_abs_bearing_x <= 0.35`
- `avg_abs_area_error_ratio <= 0.55`
- `avg_command_delta <= 1.0`

### 2. Tuned Mock Baseline

A tuned Mission 1 config was created at:

- `packages/mission_framework/configs/solitary_sentinel_tuned.yaml`

In the clean mock environment, the baseline values already passed, so the tuned
config serves as the validated mock baseline rather than a materially retuned
controller.

### 3. Robustness And Variability Benchmarking

The mock setup was extended to evaluate how the Mission 1 controller behaves
under controlled disturbance:

- sensor noise levels
- actuator noise levels
- multiple starting poses
- temporal detection dropout bursts
- fixed detection latency
- richer target motion profiles

This produced a repeatable mock benchmark that helps quantify where the Mission
1 baseline remains stable and where it begins to degrade.

### 4. Validation Artifacts And Reporting

The work produced:

- validation notes
- tuned config
- robustness benchmark outputs
- machine-readable summaries (`json`, `csv`)
- a human-readable summary report

## How

The Mission 1 validation flow used the following local mock pipeline:

`mock_bridge.py -> mock_detection_publisher.py -> run_mission.py`

The bridge simulates odometry and command handling, the publisher emits
synthetic detections, and the mission runner executes the configured mission.

### Main Components

- `sim_interfaces/airsim_zenoh_bridge/mock_bridge.py`
- `sim_interfaces/airsim_zenoh_bridge/mock_detection_publisher.py`
- `sim_interfaces/airsim_zenoh_bridge/run_mission.py`
- `packages/mission_framework/phases/follow_target.py`
- `packages/mission_framework/configs/solitary_sentinel.yaml`
- `packages/mission_framework/configs/solitary_sentinel_tuned.yaml`

### Validation Method

The baseline mission was run repeatedly on the mock setup and evaluated against
the configured acceptance criteria.

Three consecutive passing runs were used as the baseline completion signal.

After that, the benchmark was expanded to cover:

- 5 sensor noise levels
- 5 actuator noise levels
- multiple starting positions
- multiple target trajectories
- dropout bursts
- latency injection

Benchmark automation was run through:

- `scripts/run_mission1_robustness.py`

## Results

### Baseline Mission Validation

Mission 1 passed 3 consecutive mock validation runs without timeout or
acceptance failure.

Representative outcome:

- `track_ratio = 1.0`
- `avg_abs_bearing_x ~= 0.25`
- `avg_abs_area_error_ratio ~= 0.25`
- `avg_command_delta < 0.5`

### Robustness Benchmark

The enhanced mock benchmark showed that the controller remained broadly stable
across a wide range of disturbances, while more challenging cases exposed the
expected failure boundary.

Main observations:

- actuator noise was comparatively well tolerated
- perception-side corruption was the stronger failure driver
- more complex curved motion was harder than straight or speed-ramp motion

The most severe failures appeared under:

- `curve` trajectory
- `very_high` sensor noise

This is a useful result because it highlights where the current baseline is
strong and where the next round of tuning or architecture work should focus.

## Learnings

The most important learnings from this work were:

1. The Mission 1 controller is already stable on the clean mock baseline.
2. Perception-side degradation is a more important stressor than actuator-side
   noise in the current mock setup.
3. Motion complexity matters: curved target motion exposes controller weakness
   sooner than straight-line motion.
4. A lightweight mock benchmark is valuable because it surfaces controller
   sensitivity cheaply before moving to simulator-backed testing.

## What To Check In

Recommended check-in items:

- `packages/mission_framework/configs/solitary_sentinel_tuned.yaml`
- `docs/plans/solitary-sentinel-mock-validation.md`
- `docs/plans/mission1-solitary-sentinel-working-doc.md`
- any supporting scripts/code changes required to reproduce the benchmark

Recommended not to check in by default:

- raw transient runtime outputs under `outputs/`
- per-run logs
- generated metrics artifacts that are reproducible and not source-of-truth

The PR can instead summarize the results and refer to the validation/benchmark
docs as the durable record.

## Next Step

The mock-based Mission 1 baseline is complete and well characterized.

The natural next step is to repeat the same Mission 1 validation flow in a
Project AirSim environment so that the controller can be evaluated against
simulator-backed perception, motion, and scene complexity.
