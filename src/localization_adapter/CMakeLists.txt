cmake_minimum_required(VERSION 3.8)
project(localization_adapter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(pcl_conversions REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(icp_localization_node
  src/icp_localization_node.cpp
)

ament_target_dependencies(icp_localization_node
  ${dependencies}
)
target_link_libraries(icp_localization_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  icp_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_package()
