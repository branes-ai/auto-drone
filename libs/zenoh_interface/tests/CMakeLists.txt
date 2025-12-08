# libs/zenoh_interface/tests/CMakeLists.txt

add_executable(zenoh_interface_tests
    ZenohInterfaceTest.cpp
)

target_link_libraries(zenoh_interface_tests PRIVATE
    zenoh_interface
    Catch2::Catch2WithMain
)

catch_discover_tests(zenoh_interface_tests)
