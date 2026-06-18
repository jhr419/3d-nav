cmake_minimum_required(VERSION 3.8)
project(nav3d_global_planning)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/3dnav_global_planning)
install(FILES resource/3dnav_global_planning DESTINATION share/ament_index/resource_index/packages)

ament_package()
