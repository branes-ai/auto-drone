# demos/05_behavior_tree/CMakeLists.txt
#
# Behavior Tree Mission Runner Demo
#
# This demo loads and executes behavior trees that orchestrate
# mission phases via Zenoh RPC to the Python phase server.

add_executable(bt_mission_runner
    bt_mission_runner.cpp
)

target_link_libraries(bt_mission_runner PRIVATE
    behavior_tree
    zenoh_interface
    data_types
)

# Copy tree files to output directory
add_custom_command(TARGET bt_mission_runner POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:bt_mission_runner>/trees
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/libs/behavior_tree/trees
        $<TARGET_FILE_DIR:bt_mission_runner>/trees
    COMMENT "Copying behavior tree XML files"
)
