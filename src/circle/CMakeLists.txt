cmake_minimum_required(VERSION 3.5)
project(circle)
 
# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()
 
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
 
# Add circle1 executable
add_executable(circle1 src/circle1.cpp)
ament_target_dependencies(circle1 rclcpp px4_msgs)
install(TARGETS circle1 DESTINATION lib/${PROJECT_NAME})

# Add circle2_drones executable
add_executable(circle2_drones src/circle2_drones.cpp)
ament_target_dependencies(circle2_drones rclcpp px4_msgs)
install(TARGETS circle2_drones DESTINATION lib/${PROJECT_NAME})
 
# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)
 
ament_package()