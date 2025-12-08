# demos/02_waypoint_following/CMakeLists.txt

# Waypoint Manager - subscribes to odometry and waypoints, publishes velocity commands
add_executable(waypoint_manager
    WaypointManager.cpp
)
target_link_libraries(waypoint_manager PRIVATE
    zenoh_interface
    data_types
    control_algorithms
)

# Waypoint Publisher - utility to publish waypoint sequences for testing
add_executable(waypoint_publisher
    WaypointPublisher.cpp
)
target_link_libraries(waypoint_publisher PRIVATE
    zenoh_interface
    data_types
)

# Simulated Drone - subscribes to velocity commands, publishes odometry
add_executable(simulated_drone
    SimulatedDrone.cpp
)
target_link_libraries(simulated_drone PRIVATE
    zenoh_interface
    data_types
)
