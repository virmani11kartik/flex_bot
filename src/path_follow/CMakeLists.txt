cmake_minimum_required(VERSION 3.8)
project(path_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

set(CMAKE_CXX_STANDARD 17)

# ── Dependencies ────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# nanoflann is header-only; find it or fall back to system include
find_package(nanoflann QUIET)
if(NOT nanoflann_FOUND)
  # Try common system paths
  find_path(NANOFLANN_INCLUDE_DIR nanoflann.hpp
    PATHS /usr/include /usr/local/include /opt/ros/$ENV{ROS_DISTRO}/include
    PATH_SUFFIXES nanoflann)
  if(NOT NANOFLANN_INCLUDE_DIR)
    message(FATAL_ERROR "nanoflann not found. Install with: sudo apt install libnanoflann-dev")
  endif()
  message(STATUS "nanoflann found at: ${NANOFLANN_INCLUDE_DIR}")
endif()

# Common ament deps list (reused across targets)
set(COMMON_DEPS
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ── prm_builder ─────────────────────────────────────────────────────────────
add_executable(prm_builder src/prm_builder.cpp)
ament_target_dependencies(prm_builder ${COMMON_DEPS})
if(nanoflann_FOUND)
  target_link_libraries(prm_builder nanoflann::nanoflann)
else()
  target_include_directories(prm_builder PRIVATE ${NANOFLANN_INCLUDE_DIR})
endif()

# ── astar_search ────────────────────────────────────────────────────────────
add_executable(astar_search src/astar_search.cpp)
ament_target_dependencies(astar_search ${COMMON_DEPS})

# ── waypoint_controller ──────────────────────────────────────────────────────
add_executable(waypoint_controller src/waypoint_controller.cpp)
ament_target_dependencies(waypoint_controller ${COMMON_DEPS})

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS
  prm_builder
  astar_search
  waypoint_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
