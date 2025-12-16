# libs/data_types/CMakeLists.txt

add_library(data_types STATIC
    ImageData.cpp
    ImageData.hpp
    Odometry.cpp
    Odometry.hpp
    Waypoint.cpp
    Waypoint.hpp
    VelocityCommand.cpp
    VelocityCommand.hpp
    ObjectDetection.cpp
    ObjectDetection.hpp
    ProximityData.cpp
    ProximityData.hpp
)

target_link_libraries(data_types PUBLIC
    ${OpenCV_LIBS}
)

target_include_directories(data_types PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    ${OpenCV_INCLUDE_DIRS}
)

# Add tests if testing is enabled
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
