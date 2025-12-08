# demos/01_sensor_streaming/CMakeLists.txt

# Mock Publisher - generates synthetic sensor data
add_executable(mock_publisher
    MockPublisher.cpp
)

target_link_libraries(mock_publisher PRIVATE
    zenoh_interface
    data_types
    ${OpenCV_LIBS}
)

# Viewer Node - subscribes and displays sensor data
add_executable(viewer_node
    ViewerNode.cpp
)

target_link_libraries(viewer_node PRIVATE
    zenoh_interface
    data_types
    ${OpenCV_LIBS}
)
