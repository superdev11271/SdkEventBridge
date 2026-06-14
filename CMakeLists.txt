cmake_minimum_required(VERSION 3.10)
project(SdkEventBridge VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(unitree_sdk2 REQUIRED)

add_library(sdk_event_bridge_lib STATIC
    src/sdk_event_bridge.cpp
    src/motion_switcher_bridge.cpp
    src/cmd_vel_publisher.cpp
    src/cmd_ctl_publisher.cpp
    src/int32_.cpp
    src/reset_subscriber.cpp
)
target_include_directories(sdk_event_bridge_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(sdk_event_bridge_lib
    PUBLIC
        unitree_sdk2
)

add_executable(sdk_event_bridge src/main.cpp)
target_link_libraries(sdk_event_bridge PRIVATE sdk_event_bridge_lib)
