cmake_minimum_required(VERSION 3.8)
project(global_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  octomap_msgs
)

add_library(global_planner_core
  src/global_planner_core.cpp
)
target_include_directories(global_planner_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(global_planner_core
  octomap_msgs
)
target_link_libraries(global_planner_core
  octomap
  octomath
)

add_executable(global_planner_node
  src/global_planner_node.cpp
)
target_link_libraries(global_planner_node
  global_planner_core
  ${PCL_LIBRARIES}
)
target_include_directories(global_planner_node
  PRIVATE
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(global_planner_node
  ${dependencies}
)

install(TARGETS
  global_planner_core
  EXPORT export_global_planner_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_global_planner_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
