# sim_interfaces/airsim_client/CMakeLists.txt
#
# Project AirSim client library for bridging Project AirSim simulator to Zenoh
#
# NOTE: This is for Project AirSim (IAMAI), not the old Microsoft AirSim.
#       See: https://github.com/iamaisim/ProjectAirSim
#
# USAGE:
#   cmake -DPROJECTAIRSIM_ROOT=/path/to/ProjectAirSim ..
#
# The PROJECTAIRSIM_ROOT should point to the Project AirSim repository root.
# The C++ client library is expected at: client/cpp/
#
# If PROJECTAIRSIM_ROOT is not set, this module will be skipped with a warning.

# Check if Project AirSim is available (prefer PROJECTAIRSIM_ROOT, fall back to AIRSIM_ROOT for compatibility)
if(NOT DEFINED PROJECTAIRSIM_ROOT AND DEFINED ENV{PROJECTAIRSIM_ROOT})
    set(PROJECTAIRSIM_ROOT $ENV{PROJECTAIRSIM_ROOT})
endif()

# Fall back to AIRSIM_ROOT for backwards compatibility
if(NOT PROJECTAIRSIM_ROOT)
    if(DEFINED AIRSIM_ROOT)
        set(PROJECTAIRSIM_ROOT ${AIRSIM_ROOT})
        message(STATUS "Using AIRSIM_ROOT as fallback (deprecated - use PROJECTAIRSIM_ROOT)")
    elseif(DEFINED ENV{AIRSIM_ROOT})
        set(PROJECTAIRSIM_ROOT $ENV{AIRSIM_ROOT})
        message(STATUS "Using ENV{AIRSIM_ROOT} as fallback (deprecated - use PROJECTAIRSIM_ROOT)")
    endif()
endif()

if(NOT PROJECTAIRSIM_ROOT)
    message(STATUS "PROJECTAIRSIM_ROOT not set - skipping airsim_client build")
    message(STATUS "  To build airsim_client, set PROJECTAIRSIM_ROOT to your Project AirSim repository")
    message(STATUS "  Example: cmake -DPROJECTAIRSIM_ROOT=/path/to/ProjectAirSim ..")
    return()
endif()

# Project AirSim client is in client/cpp/
set(PROJECTAIRSIM_CLIENT_DIR "${PROJECTAIRSIM_ROOT}/client/cpp")

if(NOT EXISTS "${PROJECTAIRSIM_CLIENT_DIR}")
    message(WARNING "PROJECTAIRSIM_ROOT=${PROJECTAIRSIM_ROOT} does not contain client/cpp - skipping airsim_client")
    message(STATUS "  Expected Project AirSim structure with client/cpp/ directory")
    return()
endif()

message(STATUS "Building airsim_client with PROJECTAIRSIM_ROOT=${PROJECTAIRSIM_ROOT}")

# Project AirSim paths
# The structure is: client/cpp/AirSimClientDLL/Include/AirSimClient/
set(PROJECTAIRSIM_INCLUDE_DIR "${PROJECTAIRSIM_CLIENT_DIR}/AirSimClientDLL/Include")
set(PROJECTAIRSIM_LIB_DIR "${PROJECTAIRSIM_CLIENT_DIR}/build")

# Find required packages
find_package(Threads REQUIRED)

# --- AirSim Client Library ---
add_library(airsim_client STATIC
    AirSimClient.cpp
    AirSimClient.hpp
)

target_include_directories(airsim_client PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# Check for Project AirSim client include directory
if(EXISTS "${PROJECTAIRSIM_INCLUDE_DIR}")
    target_include_directories(airsim_client SYSTEM PUBLIC
        ${PROJECTAIRSIM_INCLUDE_DIR}
    )
    target_compile_definitions(airsim_client PRIVATE PROJECTAIRSIM_AVAILABLE)
    message(STATUS "  Found Project AirSim includes at: ${PROJECTAIRSIM_INCLUDE_DIR}")
else()
    message(STATUS "  Project AirSim includes not found at: ${PROJECTAIRSIM_INCLUDE_DIR}")
    message(STATUS "  Building without Project AirSim client - stub implementation only")
endif()

# Link to our data types
target_link_libraries(airsim_client PUBLIC
    data_types
)

# Platform-specific Project AirSim library paths
if(WIN32)
    set(PROJECTAIRSIM_LIB_SEARCH_DIR "${PROJECTAIRSIM_LIB_DIR}/Release")
else()
    set(PROJECTAIRSIM_LIB_SEARCH_DIR "${PROJECTAIRSIM_LIB_DIR}")
endif()

# Find Project AirSim client library
# The library name may vary - try common variants
find_library(PROJECTAIRSIM_CLIENT_LIB
    NAMES AirSimClient airsim_client ProjectAirSimClient
    PATHS ${PROJECTAIRSIM_LIB_SEARCH_DIR}
    NO_DEFAULT_PATH
)

if(PROJECTAIRSIM_CLIENT_LIB)
    target_link_libraries(airsim_client PRIVATE
        ${PROJECTAIRSIM_CLIENT_LIB}
        Threads::Threads
    )
    message(STATUS "  Found Project AirSim client library: ${PROJECTAIRSIM_CLIENT_LIB}")
else()
    message(STATUS "  Project AirSim client library not found - build the client first")
    message(STATUS "  Expected location: ${PROJECTAIRSIM_LIB_SEARCH_DIR}")
    message(STATUS "  Build with: cd ${PROJECTAIRSIM_CLIENT_DIR} && mkdir build && cd build && cmake .. && make")

    # Just link threading for now
    target_link_libraries(airsim_client PRIVATE
        Threads::Threads
    )
endif()

# Compiler features
target_compile_features(airsim_client PUBLIC cxx_std_20)

# --- AirSim Bridge Executable ---
add_executable(airsim_bridge
    AirSimBridge.cpp
)

target_link_libraries(airsim_bridge PRIVATE
    airsim_client
    zenoh_interface
    data_types
)

# Installation
install(TARGETS airsim_bridge RUNTIME DESTINATION bin)
