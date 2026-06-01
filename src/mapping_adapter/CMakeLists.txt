cmake_minimum_required(VERSION 3.8)
project(mapping_adapter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(mapping_adapter_node
  src/mapping_adapter_node.cpp
)

ament_target_dependencies(mapping_adapter_node
  ${dependencies}
)
target_link_libraries(mapping_adapter_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  mapping_adapter_node
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
