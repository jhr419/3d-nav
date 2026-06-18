cmake_minimum_required(VERSION 3.8)
project(nav3d_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/3dnav_bringup
)

install(FILES
  resource/3dnav_bringup
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()
