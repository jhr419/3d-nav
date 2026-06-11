cmake_minimum_required(VERSION 3.8)
project(sc_pgo)

if(CMAKE_CXX_STANDARD LESS 17)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

message(STATUS "GTSAM_INCLUDE_DIR: ${GTSAM_INCLUDE_DIR}")
message(STATUS "GTSAM_INCLUDE_DIRS: ${GTSAM_INCLUDE_DIRS}")
message(STATUS "GTSAM_LIBRARIES: ${GTSAM_LIBRARIES}")

set(SC_PGO_GTSAM_LIBRARY "")
if(TARGET gtsam)
  get_target_property(SC_PGO_GTSAM_LIBRARY gtsam IMPORTED_LOCATION_RELEASE)
  if(NOT SC_PGO_GTSAM_LIBRARY)
    get_target_property(SC_PGO_GTSAM_LIBRARY gtsam IMPORTED_LOCATION)
  endif()
endif()
if(NOT SC_PGO_GTSAM_LIBRARY)
  find_library(SC_PGO_GTSAM_LIBRARY
    NAMES gtsam
    HINTS ${GTSAM_LIBRARY_DIRS} /usr/local/lib /usr/lib/x86_64-linux-gnu
  )
endif()
if(NOT SC_PGO_GTSAM_LIBRARY)
  message(FATAL_ERROR "Unable to find libgtsam.so. Install libgtsam-dev or set GTSAM_LIBRARY_DIRS.")
endif()
message(STATUS "SC_PGO_GTSAM_LIBRARY: ${SC_PGO_GTSAM_LIBRARY}")

set(SC_PGO_PRIVATE_GTSAM_DIR "${CMAKE_CURRENT_BINARY_DIR}/gtsam_runtime")
file(MAKE_DIRECTORY "${SC_PGO_PRIVATE_GTSAM_DIR}")
configure_file("${SC_PGO_GTSAM_LIBRARY}" "${SC_PGO_PRIVATE_GTSAM_DIR}/libgtsam.so.4" COPYONLY)

set(SC_PGO_GTSAM_BUNDLED_EIGEN_INCLUDE "")
foreach(_gtsam_include_dir ${GTSAM_INCLUDE_DIR} ${GTSAM_INCLUDE_DIRS} /usr/local/include)
  if(EXISTS "${_gtsam_include_dir}/gtsam/3rdparty/Eigen/Eigen/Core")
    set(SC_PGO_GTSAM_BUNDLED_EIGEN_INCLUDE "${_gtsam_include_dir}/gtsam/3rdparty/Eigen")
    break()
  endif()
endforeach()
message(STATUS "SC_PGO_GTSAM_BUNDLED_EIGEN_INCLUDE: ${SC_PGO_GTSAM_BUNDLED_EIGEN_INCLUDE}")

add_library(scan_context_lib
  src/scan_context.cpp
)
target_include_directories(scan_context_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
if(SC_PGO_GTSAM_BUNDLED_EIGEN_INCLUDE)
  target_include_directories(scan_context_lib BEFORE PUBLIC
    ${SC_PGO_GTSAM_BUNDLED_EIGEN_INCLUDE}
  )
endif()
target_link_libraries(scan_context_lib
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

add_executable(sc_pgo_node
  src/sc_pgo_node.cpp
)
target_include_directories(sc_pgo_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)
if(SC_PGO_GTSAM_BUNDLED_EIGEN_INCLUDE)
  target_include_directories(sc_pgo_node BEFORE PRIVATE
    ${SC_PGO_GTSAM_BUNDLED_EIGEN_INCLUDE}
  )
endif()
target_include_directories(sc_pgo_node PRIVATE
  ${GTSAM_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}/gtsam/3rdparty/SuiteSparse_config
  ${GTSAM_INCLUDE_DIR}/gtsam/3rdparty/CCOLAMD
  ${GTSAM_INCLUDE_DIR}/gtsam/3rdparty/metis
)

target_link_libraries(sc_pgo_node
  scan_context_lib
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${SC_PGO_GTSAM_LIBRARY}
)
set_target_properties(sc_pgo_node PROPERTIES
  BUILD_RPATH "${SC_PGO_PRIVATE_GTSAM_DIR}"
  INSTALL_RPATH "$ORIGIN/gtsam_runtime"
)

ament_target_dependencies(sc_pgo_node
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
)

install(TARGETS
  scan_context_lib
  sc_pgo_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES "${SC_PGO_PRIVATE_GTSAM_DIR}/libgtsam.so.4"
  DESTINATION lib/${PROJECT_NAME}/gtsam_runtime
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(scan_context_lib)
ament_export_dependencies(
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
