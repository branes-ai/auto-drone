#!/usr/bin/env bash


To run the demo:

# Terminal 1: Start simulated drone
bash  ./build/linux-release/demos/02_waypoint_following/simulated_drone

# Terminal 2: Start waypoint manager
bash  ./build/linux-release/demos/02_waypoint_following/waypoint_manager

# Terminal 3: Publish waypoints
./build/linux-release/demos/02_waypoint_following/waypoint_publisher --pattern square --size 5

