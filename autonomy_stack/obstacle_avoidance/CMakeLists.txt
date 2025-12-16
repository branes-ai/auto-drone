# autonomy_stack/obstacle_avoidance/CMakeLists.txt

# --- Command Arbiter Library ---
add_library(command_arbiter STATIC
    CommandArbiter.cpp
    CommandArbiter.hpp
)

target_include_directories(command_arbiter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(command_arbiter PUBLIC
    data_types
)

target_compile_features(command_arbiter PUBLIC cxx_std_20)
