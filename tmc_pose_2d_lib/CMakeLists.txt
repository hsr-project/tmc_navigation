cmake_minimum_required(VERSION 3.5)
project(tmc_pose_2d_lib)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tmc_navigation_msgs REQUIRED)

add_library(pose_2d SHARED
  src/pose_2d.cpp
  src/distance_map.cpp)
target_compile_definitions(pose_2d
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(pose_2d PUBLIC -Wall)
ament_target_dependencies(pose_2d rclcpp)
target_include_directories(pose_2d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(ros_if SHARED
  src/ros_if.cpp)

target_compile_definitions(ros_if
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(ros_if PUBLIC -Wall)
ament_target_dependencies(ros_if
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  tmc_navigation_msgs
)

ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  tmc_navigation_msgs
)

target_include_directories(ros_if
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_environment_hooks(
  "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

target_link_libraries(ros_if pose_2d)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS pose_2d
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(TARGETS ros_if
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(pose_2d_test test/pose_2d-test.cpp)
  target_link_libraries(pose_2d_test pose_2d)

  ament_add_gtest(distance_map_test test/distance_map-test.cpp)
  target_link_libraries(distance_map_test pose_2d)

  find_package(Eigen3 REQUIRED)
  ament_add_gtest(pose_2d_eigen_compare_test test/pose_2d_eigen_compare-test.cpp)
  target_link_libraries(pose_2d_eigen_compare_test pose_2d)
  target_include_directories(pose_2d_eigen_compare_test
    PUBLIC
      $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  )
endif()
ament_package()
