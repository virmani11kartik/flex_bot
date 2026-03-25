cmake_minimum_required(VERSION 3.8)
project(positioning_system)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(DEPS rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs 
         tf2 tf2_ros ament_index_cpp)

# ── pgv_dock_controller ───────────────────────────────────────────────────────
add_executable(pgv_dock_controller src/pgv_dock_controller.cpp)
ament_target_dependencies(pgv_dock_controller ${DEPS})

install(TARGETS pgv_dock_controller
  DESTINATION lib/${PROJECT_NAME})

# ── Python scripts ────────────────────────────────────────────────────────────
install(PROGRAMS
  src/tag_registration.py
  src/waypoint_cli.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
