cmake_minimum_required(VERSION 3.8)
project(octo_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(jie_map_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(jie_path_node src/jie_path_node.cpp)
ament_target_dependencies(jie_path_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  jie_map_msgs
  octomap_msgs
  visualization_msgs
)
target_link_libraries(jie_path_node octomap)

add_executable(d1_controller src/d1_controller.cpp)
target_include_directories(d1_controller PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(d1_controller
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(d1_controller ${OpenCV_LIBS})

add_executable(test_map_to_odom_tf_node src/test_map_to_odom_tf_node.cpp)
ament_target_dependencies(test_map_to_odom_tf_node
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(icp_pose_to_start_point_node src/icp_pose_to_start_point_node.cpp)
ament_target_dependencies(icp_pose_to_start_point_node
  geometry_msgs
  rclcpp
)

add_executable(jie_3d_global_planner_adapter_node src/jie_3d_global_planner_adapter_node.cpp)
ament_target_dependencies(jie_3d_global_planner_adapter_node
  geometry_msgs
  nav_msgs
  octomap_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(jie_3d_global_planner_adapter_node octomap)

install(TARGETS
  jie_path_node
  d1_controller
  test_map_to_odom_tf_node
  icp_pose_to_start_point_node
  jie_3d_global_planner_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
