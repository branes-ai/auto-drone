# auto-drone/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

####
# Set project variables
if(NOT DEFINED AUTODRONE_VERSION_MAJOR)
  set(AUTODRONE_VERSION_MAJOR 0)
endif()
if(NOT DEFINED AUTODRONE_VERSION_MINOR)
  set(AUTODRONE_VERSION_MINOR 1)
endif()
if(NOT DEFINED AUTODRONE_VERSION_PATCH)
  set(AUTODRONE_VERSION_PATCH 1)
endif()

cmake_policy(SET CMP0048 NEW) # The project() command manages VERSION variables
cmake_policy(SET CMP0077 NEW) # option() honors normal variables

project(autodrone
	DESCRIPTION "Autonomous Drone Software Development Kit"
	VERSION "${AUTODRONE_VERSION_MAJOR}.${AUTODRONE_VERSION_MINOR}.${AUTODRONE_VERSION_PATCH}" 
	LANGUAGES C CXX ASM
	HOMEPAGE_URL "https://github.com/branes-ai/auto-drone")

# Require C++20 or newer for modern Zenoh bindings and features
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Add cmake/ directory to module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Include your helpers early
include(helpers)

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

option(AUTODRONE_USE_FOLDERS "Enable solution folders in Visual Studio, disable for Express"   ON)
if (AUTODRONE_USE_FOLDERS) 
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif()

# --- Core Libraries ---
add_subdirectory(libs/data_types)
add_subdirectory(libs/zenoh_interface)
add_subdirectory(libs/control_algorithms)
add_subdirectory(libs/behavior_tree)          # BehaviorTree.CPP integration

# --- Simulation/Platform Interfaces ---
add_subdirectory(sim_interfaces/airsim_client)       # Requires AIRSIM_ROOT
add_subdirectory(sim_interfaces/airsim_zenoh_bridge) # Python bridge + C++ test client
# add_subdirectory(sim_interfaces/isaac_client)
# add_subdirectory(platforms/drone_platform)

# --- Autonomy Stack ---
add_subdirectory(autonomy_stack/object_tracking)
add_subdirectory(autonomy_stack/obstacle_avoidance)  # Phase 4
# add_subdirectory(autonomy_stack/visual_slam)      # Phase 5
# add_subdirectory(autonomy_stack/visual_nav)       # Phase 6

# --- Demonstrations ---
add_subdirectory(demos/01_sensor_streaming)
add_subdirectory(demos/02_waypoint_following)
add_subdirectory(demos/03_object_tracking)
add_subdirectory(demos/04_obstacle_avoidance)  # Phase 4
add_subdirectory(demos/05_behavior_tree)       # Behavior Tree mission runner
