# auto-drone/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)
project(AUTO_DRONE LANGUAGES CXX)

# Require C++20 or newer for modern Zenoh bindings and features
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# --- Testing Support ---
include(CTest)

# Fetch Catch2 if testing is enabled
if(BUILD_TESTING)
    include(FetchContent)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.5.0
    )
    FetchContent_MakeAvailable(Catch2)
    list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
    include(Catch)
endif()

# --- Find Third-Party Dependencies ---
find_package(zenohc REQUIRED)
find_package(OpenCV REQUIRED)

# --- Core Libraries ---
add_subdirectory(libs/data_types)
add_subdirectory(libs/zenoh_interface)
# add_subdirectory(libs/control_algorithms)  # Phase 2

# --- Simulation/Platform Interfaces ---
# add_subdirectory(sim_interfaces/airsim_client)  # After mock publisher works
# add_subdirectory(sim_interfaces/isaac_client)
# add_subdirectory(platforms/drone_platform)

# --- Autonomy Stack ---
# add_subdirectory(autonomy_stack/visual_slam)      # Phase 5
# add_subdirectory(autonomy_stack/object_tracking)  # Phase 3
# add_subdirectory(autonomy_stack/visual_nav)       # Phase 6

# --- Demonstrations ---
add_subdirectory(demos/01_sensor_streaming)
# add_subdirectory(demos/02_object_tracking)    # Phase 3
# add_subdirectory(demos/03_obstacle_avoidance) # Phase 4
