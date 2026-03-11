cmake_minimum_required(VERSION 3.5)
project(drone_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_executable(drone_tracker_node src/drone_tracker_node.cpp)
ament_target_dependencies(drone_tracker_node rclcpp px4_msgs vision_msgs)

install(TARGETS drone_tracker_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

