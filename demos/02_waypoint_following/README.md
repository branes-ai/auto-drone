# Waypoint following

● The waypoint demo executables are now ready to connect to a remote Zenoh endpoint. Here's a summary of what was done and how to test:

## Summary

  I've added --connect option to both waypoint demo executables so they can connect to the AirSim bridge running on Windows.

## Files Modified:

  - demos/02_waypoint_following/WaypointManager.cpp - Added --connect <endpoint> option
  - demos/02_waypoint_following/WaypointPublisher.cpp - Added --connect <endpoint> option

## How to Test with AirSim Bridge

  On Windows (assuming the bridge is already running with Project AirSim):
  > python airsim_zenoh_bridge.py --scene scene_basic_drone.jsonc --zenoh-listen tcp/0.0.0.0:7447

  On Linux (two terminals):

  Terminal 1 - Start the waypoint manager (PID controller):
  ./build/linux-release/demos/02_waypoint_following/waypoint_manager --connect tcp/192.168.1.10:7447

  Terminal 2 - Send waypoints:
  # Square pattern (default 5m sides, 10m altitude)
  ./build/linux-release/demos/02_waypoint_following/waypoint_publisher --connect tcp/192.168.1.10:7447 --pattern square

  # Or circle pattern
  ./build/linux-release/demos/02_waypoint_following/waypoint_publisher --connect tcp/192.168.1.10:7447 --pattern circle --size 5 --altitude 10

  Architecture:

  Windows                          Linux
  ┌─────────────────┐              ┌─────────────────────┐
  │  Project AirSim │              │  waypoint_manager   │
  │       ↕         │              │  (PID controller)   │
  │  airsim_bridge  │◄─── Zenoh ──►│         ↓           │
  │  (tcp:7447)     │              │  waypoint_publisher │
  └─────────────────┘              └─────────────────────┘

  The waypoint_manager subscribes to:
  - robot/drone/sensor/state/odom - odometry from bridge
  - robot/drone/cmd/waypoints - waypoints from publisher

  And publishes:
  - robot/drone/cmd/velocity - velocity commands to bridge

---
The waypoint demo is working perfectly with the AirSim bridge!

  The drone successfully flew the complete square pattern:
  - WP 0 (5, 0, -10) - Reached ✓
  - WP 1 (5, 5, -10) - Reached ✓
  - WP 2 (0, 5, -10) - Reached ✓
  - WP 3 (0, 0, -10) - Reached ✓
  - Mission complete!

## Waypoint Demo Integration Complete

  The waypoint following demo is now fully integrated with the AirSim bridge:

  What Was Done:

  1. Added --connect option to waypoint_manager and waypoint_publisher executables
  2. Tested successfully - drone flew a complete 5m square pattern at 10m altitude

  Test Results:

  WP 0 (5,0,-10)  → Reached at 0.49m
  WP 1 (5,5,-10)  → Reached at 0.49m
  WP 2 (0,5,-10)  → Reached at 0.49m
  WP 3 (0,0,-10)  → Reached at 0.49m
  Mission complete!

  Usage:

```bash
  # Terminal 1: Waypoint manager (PID controller)
  ./waypoint_manager --connect tcp/192.168.1.10:7447

  # Terminal 2: Send waypoints
  ./waypoint_publisher --connect tcp/192.168.1.10:7447 --pattern square
  ./waypoint_publisher --connect tcp/192.168.1.10:7447 --pattern circle --size 5
```
