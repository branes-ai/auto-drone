# demos/04_obstacle_avoidance/CMakeLists.txt

# Mock Proximity Publisher - generates synthetic proximity sensor data
add_executable(mock_proximity_publisher
    MockProximityPublisher.cpp
)
target_link_libraries(mock_proximity_publisher PRIVATE
    zenoh_interface
    data_types
)

# Obstacle Avoidance Node - reactive avoidance from proximity data
add_executable(obstacle_avoidance_node
    ObstacleAvoidanceNode.cpp
)
target_link_libraries(obstacle_avoidance_node PRIVATE
    zenoh_interface
    data_types
    control_algorithms
)

# Command Arbiter Node - arbitrates between nominal and reactive commands
add_executable(command_arbiter_node
    CommandArbiterNode.cpp
)
target_link_libraries(command_arbiter_node PRIVATE
    zenoh_interface
    data_types
    command_arbiter
)
