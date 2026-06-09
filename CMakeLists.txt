cmake_minimum_required(VERSION 3.10)
project(SdkEventBridge VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(unitree_sdk2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(sdk_event_bridge_lib STATIC
    src/sdk_event_bridge.cpp
    src/motion_switcher_bridge.cpp
    src/cmd_vel_publisher.cpp
    src/cmd_ctl_publisher.cpp
)
target_include_directories(sdk_event_bridge_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(sdk_event_bridge_lib
    PUBLIC
        unitree_sdk2
        rclcpp::rclcpp
        geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
        std_msgs::std_msgs__rosidl_typesupport_cpp
)

add_executable(sdk_event_bridge src/main.cpp)
target_link_libraries(sdk_event_bridge PRIVATE sdk_event_bridge_lib)
