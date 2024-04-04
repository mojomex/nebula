cmake_minimum_required(VERSION 3.14)
project(topic_delay)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

find_package(sensor_msgs REQUIRED)

## Hesai
# Hw Interface
ament_auto_add_library(topic_delay_lib SHARED
        src/topic_delay.cpp
        )

rclcpp_components_register_node(topic_delay_lib
        PLUGIN "TopicDelay"
        EXECUTABLE topic_delay_exec
        )

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
        INSTALL_TO_SHARE
)

set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "rolling")
    add_compile_definitions(ROS_DISTRO_ROLLING)
elseif(${ROS_DISTRO} STREQUAL "foxy")
    add_compile_definitions(ROS_DISTRO_FOXY)
elseif(${ROS_DISTRO} STREQUAL "galactic")
    add_compile_definitions(ROS_DISTRO_GALACTIC)
elseif(${ROS_DISTRO} STREQUAL "humble")
    add_compile_definitions(ROS_DISTRO_HUMBLE)
endif()
