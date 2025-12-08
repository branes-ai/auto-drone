#!/usr/bin/env bash 
#install dependencies
#./scripts/install-deps-linux.sh 

# Configure and build
cmake --preset linux-release
cmake --build --preset linux-release

# Test
ctest --preset linux-release

# Run demo
./build/linux-release/demos/01_sensor_streaming/mock_publisher  # Terminal 1
./build/linux-release/demos/01_sensor_streaming/viewer_node     # Terminal 2 (requires display)

