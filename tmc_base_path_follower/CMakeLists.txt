cmake_minimum_required(VERSION 3.5)
project(tmc_base_path_follower)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tmc_pose_2d_lib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tmc_navigation_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(console_bridge REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/goal_checker_diff_drive.cpp
  src/goal_checker_omni.cpp
  src/nearest_path_point_searcher.cpp
  src/path_info_creator.cpp
  src/path_transit_velocity_calculator.cpp
  src/velocity_calculator_diff_drive.cpp
  src/velocity_calculator_omni.cpp
  src/base_path_follower.cpp
  src/base_path_follower_node.cpp
)

target_compile_definitions(${PROJECT_NAME}_lib
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(${PROJECT_NAME}_lib PUBLIC -Wall)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_util
  nav_msgs
  std_msgs
  std_srvs
  tf2
  tmc_navigation_msgs
  tmc_pose_2d_lib
  angles
  console_bridge
)
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

ament_environment_hooks(
  "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

add_executable(base_path_follower
  src/base_path_follower_main.cpp
)

target_include_directories(base_path_follower
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(base_path_follower
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(base_path_follower
  rclcpp
)

ament_export_dependencies(
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_util
  nav_msgs
  std_msgs
  std_srvs
  tf2
  tmc_navigation_msgs
  tmc_pose_2d_lib
  angles
  console_bridge
)

install(TARGETS
  ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS base_path_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY img
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  install(DIRECTORY test/pathdata
    DESTINATION share/${PROJECT_NAME}/test
  )
  install(DIRECTORY test/parameter
    DESTINATION share/${PROJECT_NAME}/test
  )

  # base_path_follower_node-omni-test
  ament_add_gtest(base_path_follower_node-omni-test test/base_path_follower_node-test.cpp
    APPEND_ENV PARAMETER_FILE=base_path_follower_node-omni-test.yaml
    TIMEOUT 300
  )
  target_link_libraries(base_path_follower_node-omni-test ${PROJECT_NAME}_lib yaml-cpp)

  # base_path_follower_node-diff_drive-test
  ament_add_gtest(base_path_follower_node-diff_drive-test test/base_path_follower_node-test.cpp
    APPEND_ENV PARAMETER_FILE=base_path_follower_node-diff_drive-test.yaml
    TIMEOUT 300
  )
  target_link_libraries(base_path_follower_node-diff_drive-test ${PROJECT_NAME}_lib yaml-cpp)

  # base_path_follower
  ament_add_gmock(base_path_follower-test test/base_path_follower-test.cpp)
  target_link_libraries(base_path_follower-test ${PROJECT_NAME}_lib)

  # base_path_follower_factory
  ament_add_gtest(base_path_follower_factory-test test/base_path_follower_factory-test.cpp)
  target_link_libraries(base_path_follower_factory-test ${PROJECT_NAME}_lib)

  # parameter_creator-test
  ament_add_gtest(parameter_creator-test test/parameter_creator-test.cpp)
  target_link_libraries(parameter_creator-test ${PROJECT_NAME}_lib)

  # goal_checker_omni-test
  ament_add_gtest(goal_checker_omni-test
    test/goal_checker_omni-test.cpp)
  target_link_libraries(goal_checker_omni-test ${PROJECT_NAME}_lib)

  # goal_checker_diff_drive-test
  ament_add_gtest(goal_checker_diff_drive-test
    test/goal_checker_diff_drive-test.cpp)
  target_link_libraries(goal_checker_diff_drive-test ${PROJECT_NAME}_lib)

  # path_info_creator_test
  ament_add_gtest(path_info_creator_test test/path_info_creator-test.cpp)
  target_link_libraries(path_info_creator_test ${PROJECT_NAME}_lib)

  # nearest_path_point_searcher-test
  ament_add_gtest(nearest_path_point_searcher-test
    test/nearest_path_point_searcher-test.cpp)
  target_link_libraries(nearest_path_point_searcher-test ${PROJECT_NAME}_lib)

  # velocity_calculator_omni-test
  ament_add_gtest(velocity_calculator_omni-test
    test/velocity_calculator_omni-test.cpp)
  target_link_libraries(velocity_calculator_omni-test ${PROJECT_NAME}_lib)

  # velocity_calculator_diff_drive-test
  ament_add_gtest(velocity_calculator_diff_drive-test
    test/velocity_calculator_diff_drive-test.cpp)
  target_link_libraries(velocity_calculator_diff_drive-test ${PROJECT_NAME}_lib)

  # path_transit_velocity_calculator-test
  ament_add_gtest(path_transit_velocity_calculator-test
    test/path_transit_velocity_calculator-test.cpp)
  target_link_libraries(path_transit_velocity_calculator-test ${PROJECT_NAME}_lib)
endif()
ament_package()
