cmake_minimum_required(VERSION 3.8)
project(nav3d_common)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY
  config
  DESTINATION share/3dnav_common
  OPTIONAL
)

install(FILES
  resource/3dnav_common
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()
