# libs/behavior_tree/tests/CMakeLists.txt

add_executable(test_bt_conditions
    test_conditions.cpp
)

target_link_libraries(test_bt_conditions PRIVATE
    behavior_tree
    Catch2::Catch2WithMain
)

catch_discover_tests(test_bt_conditions)
