cmake_minimum_required(VERSION 3.8)
project(nav3d_rviz_plugins)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY
  rviz
  DESTINATION share/3dnav_rviz_plugins
  OPTIONAL
)

install(FILES
  resource/3dnav_rviz_plugins
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()
