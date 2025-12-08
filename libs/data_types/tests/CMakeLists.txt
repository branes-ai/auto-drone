# libs/data_types/tests/CMakeLists.txt

add_executable(data_types_tests
    DataTypesTest.cpp
)

target_link_libraries(data_types_tests PRIVATE
    data_types
    Catch2::Catch2WithMain
)

catch_discover_tests(data_types_tests)
