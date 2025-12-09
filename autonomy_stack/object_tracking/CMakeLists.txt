# autonomy_stack/object_tracking/CMakeLists.txt

# --- Perception Library ---
add_library(perception STATIC
    PerceptionEngine.cpp
    PerceptionEngine.hpp
)

target_include_directories(perception PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(perception PUBLIC
    data_types
    ${OpenCV_LIBS}
)

target_compile_features(perception PUBLIC cxx_std_20)

# --- Perception Node Executable ---
add_executable(perception_node
    PerceptionNode.cpp
)

target_link_libraries(perception_node PRIVATE
    zenoh_interface
    data_types
    perception
)
