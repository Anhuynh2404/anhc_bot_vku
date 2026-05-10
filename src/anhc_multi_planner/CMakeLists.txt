cmake_minimum_required(VERSION 3.8)
project(anhc_multi_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ── Shared library ────────────────────────────────────────────────────────────
add_library(anhc_multi_planner SHARED
  src/anhc_multi_planner.cpp
  src/algorithms/astar.cpp
  src/algorithms/dijkstra.cpp
  src/algorithms/greedy_bfs.cpp
  src/algorithms/theta_star.cpp
  src/algorithms/jps.cpp
  src/algorithms/rrt_star.cpp
  src/algorithms/dstar_lite.cpp
)

target_include_directories(anhc_multi_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(anhc_multi_planner
  nav2_core
  nav2_costmap_2d
  nav2_util
  geometry_msgs
  nav_msgs
  std_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Tells pluginlib where to find the plugin XML at runtime
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS anhc_multi_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(anhc_multi_planner)
ament_export_dependencies(
  nav2_core
  nav2_costmap_2d
  nav2_util
  geometry_msgs
  nav_msgs
  std_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_package()
