cmake_minimum_required(VERSION 3.5)
project(tmc_odometry_switcher)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tmc_navigation_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/odometry_switcher.cpp)

target_compile_definitions(${PROJECT_NAME}_lib
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(${PROJECT_NAME}_lib PUBLIC -Wall)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  nav_msgs
  tf2_eigen
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  tmc_navigation_msgs
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_environment_hooks(
  "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

add_executable(odometry_switcher
  src/odometry_switcher_main.cpp
)
target_include_directories(odometry_switcher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(odometry_switcher
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(odometry_switcher
  rclcpp
  nav_msgs
  tf2_eigen
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  tmc_navigation_msgs
)

ament_export_dependencies(
  rclcpp
  nav_msgs
  tf2_eigen
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  tmc_navigation_msgs
)

install(TARGETS
  ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS odometry_switcher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  install(DIRECTORY test/parameter
    DESTINATION share/${PROJECT_NAME}/test
  )

  find_package(ament_cmake_gtest REQUIRED)
  find_package(angles REQUIRED)

  ament_add_gtest(odometry_switcher-test test/odometry_switcher-test.cpp)
  target_link_libraries(odometry_switcher-test ${PROJECT_NAME}_lib)
  ament_target_dependencies(odometry_switcher-test angles)
endif()
ament_package()
