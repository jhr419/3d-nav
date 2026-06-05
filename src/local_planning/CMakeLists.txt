cmake_minimum_required(VERSION 3.8)
project(local_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_go REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(dwa_3d_local_planner
  src/dynamic_obstacle_layer.cpp
  src/dwa_3d_local_planner.cpp
)
target_include_directories(dwa_3d_local_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(dwa_3d_local_planner
  octomap
)
ament_target_dependencies(dwa_3d_local_planner
  sensor_msgs
)

add_executable(local_planner_node
  src/local_planner_node.cpp
)
target_include_directories(local_planner_node
  PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(local_planner_node
  dwa_3d_local_planner
  octomap
)
ament_target_dependencies(local_planner_node
  geometry_msgs
  nav_msgs
  octomap_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  unitree_go
  visualization_msgs
)

install(TARGETS
  dwa_3d_local_planner
  local_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  docs
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(dwa_3d_local_planner)
ament_export_dependencies(
  Eigen3
  geometry_msgs
  nav_msgs
  octomap_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  unitree_go
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
