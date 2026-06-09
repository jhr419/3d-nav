cmake_minimum_required(VERSION 3.8)
project(sc_pgo)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree)
find_package(GTSAM REQUIRED QUIET)

if(TARGET gtsam)
  get_target_property(GTSAM_IMPORTED_LOCATION_RELEASE gtsam IMPORTED_LOCATION_RELEASE)
  get_target_property(GTSAM_IMPORTED_LOCATION gtsam IMPORTED_LOCATION)
endif()

if(GTSAM_IMPORTED_LOCATION_RELEASE)
  set(GTSAM_LIBRARY "${GTSAM_IMPORTED_LOCATION_RELEASE}")
elseif(GTSAM_IMPORTED_LOCATION)
  set(GTSAM_LIBRARY "${GTSAM_IMPORTED_LOCATION}")
else()
  find_library(GTSAM_LIBRARY NAMES gtsam PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu)
endif()

set(dependencies
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

add_executable(sc_pgo_node_bin
  src/sc_pgo_node.cpp
  src/scan_context.cpp
)

target_include_directories(sc_pgo_node_bin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIRS}
)

get_filename_component(GTSAM_INCLUDE_DIR_NORMALIZED "${GTSAM_INCLUDE_DIR}" REALPATH)
set(GTSAM_BUNDLED_EIGEN_DIR "${GTSAM_INCLUDE_DIR_NORMALIZED}/gtsam/3rdparty/Eigen")
if(EXISTS "${GTSAM_BUNDLED_EIGEN_DIR}/Eigen/Core")
  target_include_directories(sc_pgo_node_bin BEFORE PUBLIC
    ${GTSAM_BUNDLED_EIGEN_DIR}
  )
endif()

foreach(GTSAM_THIRDPARTY_DIR
    "${GTSAM_INCLUDE_DIR_NORMALIZED}/gtsam/3rdparty/SuiteSparse_config"
    "${GTSAM_INCLUDE_DIR_NORMALIZED}/gtsam/3rdparty/CCOLAMD"
    "${GTSAM_INCLUDE_DIR_NORMALIZED}/gtsam/3rdparty/metis")
  if(EXISTS "${GTSAM_THIRDPARTY_DIR}")
    target_include_directories(sc_pgo_node_bin PUBLIC ${GTSAM_THIRDPARTY_DIR})
  endif()
endforeach()

ament_target_dependencies(sc_pgo_node_bin ${dependencies})

target_link_libraries(sc_pgo_node_bin
  ${PCL_LIBRARIES}
)

if(GTSAM_LIBRARY)
  target_link_libraries(sc_pgo_node_bin ${GTSAM_LIBRARY})
elseif(GTSAM_LIBRARIES)
  target_link_libraries(sc_pgo_node_bin ${GTSAM_LIBRARIES})
else()
  target_link_libraries(sc_pgo_node_bin gtsam)
endif()

set_target_properties(sc_pgo_node_bin PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

if(GTSAM_LIBRARY AND EXISTS "${GTSAM_LIBRARY}")
  get_filename_component(GTSAM_LIBRARY_NAME "${GTSAM_LIBRARY}" NAME)
  set(SC_PGO_PRIVATE_GTSAM_SONAME "libgtsam.so.4")
endif()

install(TARGETS
  sc_pgo_node_bin
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sc_pgo_node
  DESTINATION lib/${PROJECT_NAME}
)

if(GTSAM_LIBRARY AND EXISTS "${GTSAM_LIBRARY}")
  install(FILES
    ${GTSAM_LIBRARY}
    DESTINATION lib/${PROJECT_NAME}
  )
  install(CODE
    "file(REMOVE \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${SC_PGO_PRIVATE_GTSAM_SONAME}\")\nexecute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink \"${GTSAM_LIBRARY_NAME}\" \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${SC_PGO_PRIVATE_GTSAM_SONAME}\")"
  )
endif()

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_package()
