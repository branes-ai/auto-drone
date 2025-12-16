# libs/control_algorithms/CMakeLists.txt

add_library(control_algorithms STATIC
    PIDController.cpp
    PIDController.hpp
    TargetTracker.cpp
    TargetTracker.hpp
    ReactiveAvoidance.cpp
    ReactiveAvoidance.hpp
)

target_include_directories(control_algorithms PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# Link to data_types for ProximityData
target_link_libraries(control_algorithms PUBLIC data_types)

# C++20 for modern features
target_compile_features(control_algorithms PUBLIC cxx_std_20)

# Add tests if testing is enabled
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
