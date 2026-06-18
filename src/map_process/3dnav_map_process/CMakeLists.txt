cmake_minimum_required(VERSION 3.8)
project(nav3d_map_process)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/3dnav_map_process)
install(FILES resource/3dnav_map_process DESTINATION share/ament_index/resource_index/packages)

ament_package()
