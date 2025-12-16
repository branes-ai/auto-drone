# libs/control_algorithms/tests/CMakeLists.txt

add_executable(control_algorithms_tests
    PIDControllerTest.cpp
    ReactiveAvoidanceTest.cpp
)

target_link_libraries(control_algorithms_tests PRIVATE
    control_algorithms
    Catch2::Catch2WithMain
)

catch_discover_tests(control_algorithms_tests)
