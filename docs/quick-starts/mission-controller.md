# Mission Controller Quick Start

This is the simplest local path to start working on the Mission 1 controller scaffold.

## What this covers

- Mission framework runtime
- Mission 1 baseline controller work
- Local mock or Zenoh-based testing

## Install

From the repo root:

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r sim_interfaces\airsim_zenoh_bridge\requirements.txt
pip install -r packages\mission_framework\requirements.txt
```

## Python path

The mission framework imports `data_types` from `sim_interfaces/airsim_zenoh_bridge`, so add both package roots to `PYTHONPATH` when running scripts directly.

```powershell
$env:PYTHONPATH = "$PWD\packages;$PWD\sim_interfaces\airsim_zenoh_bridge"
```

## Run the baseline mission

The current entry point is `run_mission.py`.

```powershell
python sim_interfaces\airsim_zenoh_bridge\run_mission.py --connect tcp/127.0.0.1:7447 --config packages\mission_framework\configs\orange_ball.yaml
```

If you do not have a live Zenoh source yet, start by using a mock bridge or a mock perception publisher first.

## Run Mission 1 (Solitary Sentinel) locally

Use 3 terminals from repo root.

Terminal 1 (mock odometry + camera):

```powershell
python sim_interfaces\airsim_zenoh_bridge\mock_bridge.py --zenoh-listen tcp/0.0.0.0:7447
```

Terminal 2 (mock target detections):

```powershell
python sim_interfaces\airsim_zenoh_bridge\mock_detection_publisher.py --connect tcp/127.0.0.1:7447 --camera front --target-class "orange ball"
```

Optional robustness run (noise + drops):

```powershell
python sim_interfaces\airsim_zenoh_bridge\mock_detection_publisher.py --connect tcp/127.0.0.1:7447 --camera front --target-class "orange ball" --drop-rate 0.1 --bearing-noise 0.05 --area-noise 0.15
```

Terminal 3 (Mission 1 controller):

```powershell
python sim_interfaces\airsim_zenoh_bridge\run_mission.py --connect tcp/127.0.0.1:7447 --config packages\mission_framework\configs\solitary_sentinel.yaml
```

Expected:
- Mission prints `COMPLETED` for phases and final `Mission SUCCEEDED` when acceptance passes.
- Metrics file is generated at:
  - `outputs/solitary_sentinel_baseline_metrics.json`

## Suggested Mission 1 starting point

- Subscribe to a single target stream
- Maintain a simple internal target state
- Publish follow or safe-distance commands
- Log target distance, command rate, and target loss events
