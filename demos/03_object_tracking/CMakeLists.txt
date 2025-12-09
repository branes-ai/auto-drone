# demos/03_object_tracking/CMakeLists.txt

# Mock Target Publisher - generates images with a moving colored target
add_executable(mock_target_publisher
    MockTargetPublisher.cpp
)
target_link_libraries(mock_target_publisher PRIVATE
    zenoh_interface
    data_types
    ${OpenCV_LIBS}
)

# Target Tracker Node - visual servoing to track detected objects
add_executable(target_tracker_node
    TargetTrackerNode.cpp
)
target_link_libraries(target_tracker_node PRIVATE
    zenoh_interface
    data_types
    control_algorithms
)
