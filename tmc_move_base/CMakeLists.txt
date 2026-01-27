cmake_minimum_required(VERSION 3.5)
project(tmc_move_base)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tmc_base_path_planner REQUIRED)
find_package(tmc_navigation_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/move_base.cpp
  src/logger.cpp
)

target_compile_definitions(${PROJECT_NAME}_lib
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  tmc_base_path_planner
  tmc_navigation_msgs
  nav2_util
  std_srvs
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


add_executable(move_base
  src/move_base_main.cpp
)

target_include_directories(move_base
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(move_base
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(move_base
  rclcpp
  rclcpp_action
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  tmc_base_path_planner
  tmc_navigation_msgs
  nav2_util
  std_srvs
)

ament_export_dependencies(
  rclcpp
  rclcpp_action
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  tmc_base_path_planner
  tmc_navigation_msgs
  nav2_util
  std_srvs
)

install(TARGETS
  ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS move_base
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(angles REQUIRED)

  ament_add_gtest(
    ${PROJECT_NAME}_move_base_test
    test/move_base-test.cpp
    TIMEOUT 100)
  target_link_libraries(${PROJECT_NAME}_move_base_test
                        ${PROJECT_NAME}_lib)

  ament_target_dependencies(${PROJECT_NAME}_move_base_test
    angles
  )

# TODO(syuuhei_shiro): LoggerをROS2化し、テストを作成する
if (false)
  add_rostest_gtest(${PROJECT_NAME}_logger_test
                    test/logger-test.test
                    test/logger-test.cpp
                    src/logger.cpp)
  target_link_libraries(${PROJECT_NAME}_logger_test
                        ${catkin_LIBRARIES})
  set_target_properties(${PROJECT_NAME}_logger_test
                        PROPERTIES OUTPUT_NAME logger_test)
endif()

endif()
ament_package()
