cmake_minimum_required(VERSION 3.5)
project(tmc_pose_integrator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/pose_integrator.cpp
  src/pose_integrator_node.cpp
)
target_compile_definitions(${PROJECT_NAME}_lib
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(${PROJECT_NAME}_lib PUBLIC -Wall)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
)

ament_environment_hooks(
  "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

add_executable(pose_integrator
  src/pose_integrator_main.cpp
)
target_include_directories(pose_integrator
  PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(pose_integrator
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(pose_integrator
  rclcpp
)

ament_export_dependencies(
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS
  ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS pose_integrator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  install(DIRECTORY test/parameter
    DESTINATION share/${PROJECT_NAME}/test
  )

  ament_add_gtest(
    pose_integrator_test
    test/pose_integrator-test.cpp
    TIMEOUT 100)
  target_link_libraries(pose_integrator_test ${PROJECT_NAME}_lib)
endif()
ament_package()
