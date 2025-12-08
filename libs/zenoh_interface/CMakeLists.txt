# libs/zenoh_interface/CMakeLists.txt

add_library(zenoh_interface STATIC
    ZenohInterface.cpp
    ZenohInterface.hpp
)

target_link_libraries(zenoh_interface PUBLIC
    zenohc::lib
)

target_include_directories(zenoh_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# Add tests if testing is enabled
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
