# libs/zenoh_interface/CMakeLists.txt

# Define the zenoh_interface library
add_library(zenoh_interface STATIC
    ZenohInterface.cpp
    ZenohInterface.hpp
)

# Set public dependencies (any component linking to zenoh_interface needs these)
target_link_libraries(zenoh_interface PUBLIC
    zenohc::zenohc  # The discovered Zenoh C library target
)

# Ensure the header file is available for all linkers
target_include_directories(zenoh_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)