# rnd_zenoh_embodied_ai/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)
project(RND_ZENOH_EMBODIED_AI LANGUAGES CXX)

# Require C++20 or newer for modern Zenoh bindings and features
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# --- 1. Find or Include Third-Party Dependencies (Zenoh, OpenCV, etc.) ---
# NOTE: You will need to install Zenoh-C/C++ or point to its build directory.
# For simplicity, we assume Zenoh-C/C++ is installed and discoverable.

# Check for the Zenoh C library
find_package(zenohc REQUIRED)
if (NOT zenohc_FOUND)
    message(FATAL_ERROR "Zenoh C library not found. Please ensure it is installed.")
endif()

# Example for other libraries (like OpenCV for basic image handling)
find_package(OpenCV REQUIRED)

# --- 2. Add Core Library Subdirectories ---
add_subdirectory(libs/data_types)
add_subdirectory(libs/zenoh_interface)
add_subdirectory(libs/control_algorithms)

# --- 3. Add Simulation/Platform Subdirectories ---
add_subdirectory(sim_interfaces/airsim_client)
# add_subdirectory(sim_interfaces/isaac_client) # To be enabled later
add_subdirectory(platforms/drone_platform)

# --- 4. Add Autonomy and Demonstration Subdirectories ---
add_subdirectory(autonomy_stack/visual_slam)
add_subdirectory(autonomy_stack/object_tracking)
add_subdirectory(autonomy_stack/visual_nav)
# Note: Demonstrations directories will contain their own executables
add_subdirectory(demos/01_sensor_streaming) 
add_subdirectory(demos/02_object_tracking)
add_subdirectory(demos/03_obstacle_avoidance)