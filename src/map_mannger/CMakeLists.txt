cmake_minimum_required(VERSION 3.8)
project(map_mannger)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_definitions(${PCL_DEFINITIONS})

add_executable(bin2pcd_node
  src/bin2pcd_node.cpp
)

target_include_directories(bin2pcd_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(bin2pcd_node
  rclcpp
)
target_link_libraries(bin2pcd_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  bin2pcd_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp)
ament_package()
