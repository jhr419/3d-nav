cmake_minimum_required(VERSION 3.8)
project(nav3d_robot_api)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/3dnav_robot_api)
install(FILES resource/3dnav_robot_api DESTINATION share/ament_index/resource_index/packages)

ament_package()
