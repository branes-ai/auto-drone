# libs/behavior_tree/CMakeLists.txt
#
# BehaviorTree.CPP integration for mission orchestration
#
# NOTE: We explicitly disable Groot2/ZeroMQ to keep the stack clean.
# All messaging goes through Zenoh. Future visualization can be built
# using Zenoh-based tooling.

include(FetchContent)

# Fetch BehaviorTree.CPP v4.6
FetchContent_Declare(
    behaviortree_cpp
    GIT_REPOSITORY https://github.com/BehaviorTree/BehaviorTree.CPP.git
    GIT_TAG        4.6.2
)

# Configure BT.CPP options - disable everything except core
set(BTCPP_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BTCPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BTCPP_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BTCPP_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BTCPP_GROOT_INTERFACE OFF CACHE BOOL "" FORCE)  # No ZeroMQ - we use Zenoh
set(BTCPP_SQLITE_LOGGING OFF CACHE BOOL "" FORCE)   # No SQLite dependency
set(BTCPP_ENABLE_COROUTINES OFF CACHE BOOL "" FORCE)
set(BTCPP_MANUAL_SELECTOR OFF CACHE BOOL "" FORCE)

# Disable minitrace to avoid export issues
set(BTCPP_ENABLE_MINITRACE OFF CACHE BOOL "" FORCE)

FetchContent_GetProperties(behaviortree_cpp)
if(NOT behaviortree_cpp_POPULATED)
    FetchContent_Populate(behaviortree_cpp)
    # Skip install to avoid export issues
    set(CMAKE_SKIP_INSTALL_RULES TRUE)
    add_subdirectory(${behaviortree_cpp_SOURCE_DIR} ${behaviortree_cpp_BINARY_DIR} EXCLUDE_FROM_ALL)
    set(CMAKE_SKIP_INSTALL_RULES FALSE)
endif()

# Our behavior tree library
add_library(behavior_tree STATIC
    src/bt_factory.cpp
    src/bt_conditions.cpp
    src/bt_blackboard_sync.cpp
    src/bt_phase_action.cpp
)

target_include_directories(behavior_tree PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(behavior_tree PUBLIC
    behaviortree_cpp
    zenoh_interface
    data_types
)

# Copy tree XML files to build directory for easy access
file(GLOB BT_TREE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/trees/*.xml")
if(BT_TREE_FILES)
    file(COPY ${BT_TREE_FILES} DESTINATION ${CMAKE_BINARY_DIR}/trees)
endif()

# Tests
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
