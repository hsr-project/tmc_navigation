cmake_minimum_required(VERSION 3.5)
project(tmc_base_path_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tmc_pose_2d_lib REQUIRED)
find_package(tmc_astar_lib REQUIRED)
find_package(tmc_navigation_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/base_path_planner_node.cpp
  src/grid_cells_publisher.cpp
  src/base_path_planner.cpp
  src/astar_path_planner.cpp
  src/common.cpp
  src/condition_checker.cpp
  src/path_smoother.cpp
  src/path_updater.cpp
  src/map_filter.cpp
  src/astar_path_planner_factory.cpp
  src/base_path_planner_factory.cpp
)
target_compile_definitions(${PROJECT_NAME}_lib
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  tmc_pose_2d_lib
  tmc_astar_lib
  tf2
  rclcpp_action
  geometry_msgs
  nav2_util
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  tmc_navigation_msgs
)

target_compile_options(${PROJECT_NAME}_lib PUBLIC -Wall)
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_environment_hooks(
  "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

add_executable(base_path_planner
  src/base_path_planner_main.cpp
)
target_include_directories(base_path_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(base_path_planner
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(base_path_planner
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_util
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tmc_pose_2d_lib
  tmc_astar_lib
  tmc_navigation_msgs
)

ament_export_dependencies(
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_util
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tmc_pose_2d_lib
  tmc_astar_lib
  tmc_navigation_msgs
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

install(TARGETS base_path_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(angles REQUIRED)
  find_package(console_bridge REQUIRED)

  install(DIRECTORY test/parameter
    DESTINATION share/${PROJECT_NAME}/test
  )

  # node test
  ament_add_gtest(
    base_path_planner_node-test
    test/base_path_planner_node-test.cpp
    TIMEOUT 500)
  ament_target_dependencies(base_path_planner_node-test angles)
  target_link_libraries(base_path_planner_node-test ${PROJECT_NAME}_lib)
  # for base_path_planner_factory
  ament_add_gtest(base_path_planner_factory-test
    test/base_path_planner_factory-test.cpp)
  ament_target_dependencies(base_path_planner_factory-test angles)
  target_link_libraries(base_path_planner_factory-test ${PROJECT_NAME}_lib)
  # for grid_cells_publisher
  ament_add_gtest(grid_cells_publisher-test
    test/grid_cells_publisher-test.cpp
  )
  target_link_libraries(grid_cells_publisher-test ${PROJECT_NAME}_lib)
  # for astar_path_planner_factory
  ament_add_gtest(astar_path_planner_factory-test
    test/astar_path_planner_factory-test.cpp)
  ament_target_dependencies(astar_path_planner_factory-test angles)
  target_link_libraries(astar_path_planner_factory-test ${PROJECT_NAME}_lib)

  # for base_path_planner
  ament_add_gmock(base_path_planner-test
    test/base_path_planner-test.cpp)
  ament_target_dependencies(base_path_planner-test angles)
  target_link_libraries(base_path_planner-test ${PROJECT_NAME}_lib)
  # for map_filter
  ament_add_gtest(map_filter-test
    test/map_filter-test.cpp)
  ament_target_dependencies(map_filter-test console_bridge)
  target_link_libraries(map_filter-test ${PROJECT_NAME}_lib)
  # for path_updater
  ament_add_gtest(path_updater-test
    test/path_updater-test.cpp)
  ament_target_dependencies(path_updater-test angles console_bridge)
  target_link_libraries(path_updater-test ${PROJECT_NAME}_lib)
  # for condition_checker
  ament_add_gtest(condition_checker-test
    test/condition_checker-test.cpp)
  target_link_libraries(condition_checker-test ${PROJECT_NAME}_lib)
  # for path_smoother
  ament_add_gtest(path_smoother-test
    test/path_smoother-test.cpp)
  target_link_libraries(path_smoother-test ${PROJECT_NAME}_lib)
  # for astar_path_planner
  ament_add_gtest(astar_path_planner-test
    test/astar_path_planner-test.cpp)
  ament_target_dependencies(astar_path_planner-test console_bridge)
  target_link_libraries(astar_path_planner-test ${PROJECT_NAME}_lib)
endif()
ament_package()
