cmake_minimum_required(VERSION 3.5)
project(tmc_viewpoint_controller)
# Load all dependencies required for this package
find_package(ament_cmake)
find_package(rclcpp)
find_package(geometry_msgs)
find_package(sensor_msgs)
find_package(nav_msgs)
find_package(tf2)
find_package(tf2_geometry_msgs)
find_package(tf2_ros)
find_package(trajectory_msgs)
find_package(std_srvs)
find_package(angles)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/tmc_viewpoint_controller/viewpoint_controller_node.cpp
  src/tmc_viewpoint_controller/viewpoint_to_path.cpp
  src/tmc_viewpoint_controller/viewpoint_to_tracking_target.cpp
)

target_compile_definitions(${PROJECT_NAME}_lib
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(${PROJECT_NAME}_lib PUBLIC -Wall)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  std_srvs
  angles
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

add_executable(viewpoint_controller
  src/tmc_viewpoint_controller/viewpoint_controller_main.cpp
)


target_link_libraries(viewpoint_controller
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(viewpoint_controller
  rclcpp
)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  std_srvs
  angles
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

install(TARGETS viewpoint_controller
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
  ament_add_gtest(viewpoint_controller-test
    test/viewpoint_controller-test.cpp)
  target_link_libraries(viewpoint_controller-test ${PROJECT_NAME}_lib)
endif()

ament_package()
