cmake_minimum_required(VERSION 3.8)
project(flex_bot_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(flex_bot_udp_bridge src/udp_bridge_node.cpp)
ament_target_dependencies(flex_bot_udp_bridge rclcpp std_msgs sensor_msgs)

# If you include your teleop_node.cpp in this package:
add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node rclcpp std_msgs sensor_msgs)

install(TARGETS
  flex_bot_udp_bridge
  teleop_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
