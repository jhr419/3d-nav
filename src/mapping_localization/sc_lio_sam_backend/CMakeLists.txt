cmake_minimum_required(VERSION 3.8)
project(sc_lio_sam_backend)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(GTSAM REQUIRED)

find_path(GTSAM_BUNDLED_EIGEN_INCLUDE_DIR
  NAMES Eigen/Core
  PATHS
    ${GTSAM_INCLUDE_DIR}/gtsam/3rdparty/Eigen
    /usr/local/include/gtsam/3rdparty/Eigen
  NO_DEFAULT_PATH
)

set(SC_LIO_SAM_BACKEND_SOURCES
  src/scan_context_manager.cpp
  src/keyframe_manager.cpp
  src/pose_graph_optimizer.cpp
  src/sc_lio_sam_backend_node.cpp
)

add_executable(sc_lio_sam_backend_node ${SC_LIO_SAM_BACKEND_SOURCES})
if(GTSAM_BUNDLED_EIGEN_INCLUDE_DIR)
  target_include_directories(sc_lio_sam_backend_node BEFORE PUBLIC
    ${GTSAM_BUNDLED_EIGEN_INCLUDE_DIR}
  )
endif()
target_include_directories(sc_lio_sam_backend_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(sc_lio_sam_backend_node
  ${PCL_LIBRARIES}
)

if(NOT GTSAM_BUNDLED_EIGEN_INCLUDE_DIR)
  target_link_libraries(sc_lio_sam_backend_node Eigen3::Eigen)
endif()

if(TARGET gtsam)
  target_link_libraries(sc_lio_sam_backend_node gtsam)
  get_target_property(GTSAM_ABI_LIBRARY gtsam IMPORTED_LOCATION_RELEASE)
  get_target_property(GTSAM_ABI_SONAME gtsam IMPORTED_SONAME_RELEASE)
elseif(DEFINED GTSAM_LIBRARIES)
  target_link_libraries(sc_lio_sam_backend_node ${GTSAM_LIBRARIES})
else()
  target_link_libraries(sc_lio_sam_backend_node gtsam)
endif()

ament_target_dependencies(sc_lio_sam_backend_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
  pcl_ros
)

if(GTSAM_ABI_LIBRARY AND GTSAM_ABI_SONAME)
  set(SC_LIO_SAM_BACKEND_RPATH
    "\$ORIGIN/gtsam_abi;\$ORIGIN;/usr/local/lib;/opt/ros/humble/lib"
  )
  set_target_properties(sc_lio_sam_backend_node PROPERTIES
    BUILD_RPATH "${SC_LIO_SAM_BACKEND_RPATH}"
    INSTALL_RPATH "${SC_LIO_SAM_BACKEND_RPATH}"
    BUILD_WITH_INSTALL_RPATH TRUE
  )
endif()

install(TARGETS sc_lio_sam_backend_node
  DESTINATION lib/${PROJECT_NAME}
)

if(GTSAM_ABI_LIBRARY AND GTSAM_ABI_SONAME)
  install(CODE "
    file(MAKE_DIRECTORY \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/gtsam_abi\")
    execute_process(
      COMMAND \"${CMAKE_COMMAND}\" -E create_symlink
        \"${GTSAM_ABI_LIBRARY}\"
        \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/gtsam_abi/${GTSAM_ABI_SONAME}\"
    )
  ")
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(GTSAM_ABI_LIBRARY AND GTSAM_ABI_SONAME)
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gtsam_abi_library_path.dsv"
  )
endif()

ament_package()
