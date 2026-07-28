cmake_minimum_required(VERSION 3.5)
project(tmc_safety_velocity_limiter)

find_package(rclcpp REQUIRED)
find_package(angles REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tmc_navigation_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_library(${PROJECT_NAME}_lib SHARED
  src/safety_velocity_limiter.cpp
  src/obstacle.cpp
  src/occupancy.cpp
  src/bumper_set.cpp
  src/virtual_bumper/cup_bumper.cpp
  src/virtual_bumper/ellipse_bumper.cpp
  src/virtual_bumper/occupancy_ellipse_bumper.cpp
  src/virtual_bumper/occupancy_point_bumper.cpp
  src/virtual_bumper/triangle_bumper.cpp
  src/virtual_bumper/virtual_bumper.cpp
  src/velocity_slope/fixed_slope.cpp
  src/velocity_slope/linear_slope.cpp
  src/velocity_slope/logarithm_slope.cpp
)

target_compile_options(${PROJECT_NAME}_lib PUBLIC -Wall)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  angles
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  pcl_ros
  tmc_navigation_msgs
  tf2_geometry_msgs
)


target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

add_executable(safety_velocity_limiter
  src/safety_velocity_limiter_main.cpp
)

target_include_directories(safety_velocity_limiter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(safety_velocity_limiter
  ${PROJECT_NAME}_lib
)
ament_target_dependencies(safety_velocity_limiter
  rclcpp
  angles
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  pcl_ros
  tmc_navigation_msgs
  tf2_geometry_msgs
)

ament_export_dependencies(
  rclcpp
  angles
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  pcl_ros
  tmc_navigation_msgs
)


install(TARGETS
  ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS safety_velocity_limiter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  install(DIRECTORY test/parameter
    DESTINATION share/${PROJECT_NAME}/test
  )

  ament_add_gtest(
    safety_velocity_limiter_test
    test/safety_velocity_limiter-test.cpp
    TIMEOUT 240)
  ament_target_dependencies(safety_velocity_limiter_test
    ament_index_cpp
    tf2_geometry_msgs
  )
  target_link_libraries(safety_velocity_limiter_test ${PROJECT_NAME}_lib)
endif()

ament_package()
