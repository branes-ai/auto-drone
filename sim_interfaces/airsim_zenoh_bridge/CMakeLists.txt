# sim_interfaces/airsim_zenoh_bridge/CMakeLists.txt
#
# C++ test client for the Python-based AirSim Zenoh Bridge.
# This test program can be used to verify connectivity from Linux.

# --- Test Bridge Executable (C++) ---
# This connects to the Python bridge running on Windows
add_executable(test_bridge_cpp
    test_bridge_cpp.cpp
)

target_link_libraries(test_bridge_cpp PRIVATE
    zenoh_interface
    data_types
)

target_include_directories(test_bridge_cpp PRIVATE
    ${CMAKE_SOURCE_DIR}/libs/zenoh_interface
    ${CMAKE_SOURCE_DIR}/libs/data_types
)

# Installation
install(TARGETS test_bridge_cpp RUNTIME DESTINATION bin)
