cmake_minimum_required(VERSION 3.8)
project(jie_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(jie_planning_core
  src/octomap_voxel_map.cpp
  src/global_planner_3d.cpp
)
target_include_directories(jie_planning_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(jie_planning_core
  Eigen3::Eigen
  octomap
)
ament_target_dependencies(jie_planning_core
  octomap
)

add_executable(jie_3d_global_planner_node
  src/jie_3d_global_planner_node.cpp
)
target_link_libraries(jie_3d_global_planner_node
  jie_planning_core
)
ament_target_dependencies(jie_3d_global_planner_node
  geometry_msgs
  nav_msgs
  octomap
  octomap_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS
  jie_planning_core
  jie_3d_global_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/open3d_goal_picker.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(jie_planning_core)
ament_export_dependencies(octomap)

ament_package()
