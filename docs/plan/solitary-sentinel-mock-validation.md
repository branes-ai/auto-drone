# Mission 1 Mock Validation

This note records mock-based validation of Mission 1 (`Solitary Sentinel`) using:

- `sim_interfaces/airsim_zenoh_bridge/mock_bridge.py`
- `sim_interfaces/airsim_zenoh_bridge/mock_detection_publisher.py`
- `packages/mission_framework/configs/solitary_sentinel.yaml`

The goal was to validate the acceptance workflow and confirm whether the baseline
Mission 1 parameters required tuning before moving to simulator-backed testing.

## Validation Setup

- Environment: local Zenoh-backed mock bridge + single-target mock detections
- Target: `orange ball`
- Camera: `front`
- Mission config under test: `packages/mission_framework/configs/solitary_sentinel.yaml`
- Acceptance thresholds:
  - `track_ratio >= 0.80`
  - `avg_abs_bearing_x <= 0.35`
  - `avg_abs_area_error_ratio <= 0.55`
  - `avg_command_delta <= 1.0`

## Consecutive Validation Runs

Three consecutive mock runs passed without timeout or acceptance failures.

| Run | track_ratio | avg_abs_bearing_x | avg_abs_area_error_ratio | avg_command_delta | Result |
| --- | --- | --- | --- | --- | --- |
| 1 | 1.000 | 0.256 | 0.252 | 0.346 | PASS |
| 2 | 1.000 | 0.262 | 0.252 | 0.342 | PASS |
| 3 | 1.000 | 0.254 | 0.254 | 0.408 | PASS |

Source artifacts:

- `outputs/mission1_validation/run1.metrics.json`
- `outputs/mission1_validation/run2.metrics.json`
- `outputs/mission1_validation/run3.metrics.json`

## Tuning Outcome

No configuration changes were required for the clean mock baseline.

The baseline values already met all Mission 1 acceptance criteria across 3/3
consecutive runs, so the tuned mock config is functionally identical to the
validated baseline and has been committed as:

- `packages/mission_framework/configs/solitary_sentinel_tuned.yaml`

Validated parameters:

- `desired_area: 12000`
- `area_tolerance_ratio: 0.15`
- `forward_gain: 2.0`
- `yaw_gain: 0.8`
- `max_yaw_rate: 0.6`

## Diagnostic Notes

An additional noisy diagnostic run was executed with:

- `drop-rate: 0.12`
- `bearing-noise: 0.06`
- `area-noise: 0.18`

The mission still passed acceptance on the mock pipeline, indicating that the
current baseline is stable under modest synthetic disturbance.

Diagnostic artifact:

- `outputs/mission1_validation/diagnostic_noisy.metrics.json`

## Failure Modes And Recommended Mitigations

The clean mock runs did not fail, but the following sensitivities remain the
expected first tuning levers when the mission is moved to richer simulator data:

1. If `track_ratio` drops:
   - Likely causes: detector dropouts, stale detections, camera angle mismatch
   - First mitigations: increase `detection_max_age_s` slightly, lower altitude if the target is too small, verify target contrast and camera framing

2. If `avg_abs_bearing_x` rises:
   - Likely causes: yaw controller too weak or target moving laterally too fast
   - First mitigations: increase `yaw_gain` carefully, then adjust `max_yaw_rate` if the controller remains sluggish

3. If `avg_abs_area_error_ratio` rises:
   - Likely causes: poor stand-off target area, overly aggressive or too-soft forward controller
   - First mitigations: retune `desired_area`, `forward_gain`, and `area_tolerance_ratio`

4. If `avg_command_delta` rises:
   - Likely causes: over-aggressive gains or noisy detections causing oscillation
   - First mitigations: reduce `forward_gain` and/or `yaw_gain`, widen `area_tolerance_ratio`, or add smoothing if simulator data is much noisier

## Next Step

The Mission 1 acceptance harness is now validated on mock data. The next step is
to run the same mission against a Project AirSim scene and repeat the same
acceptance process using simulator-backed perception and motion.
