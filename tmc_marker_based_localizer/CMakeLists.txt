cmake_minimum_required(VERSION 3.5)
project(tmc_marker_based_localizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tmc_eigen_bridge REQUIRED)
find_package(tmc_vision_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/marker_based_localizer.cpp
)
target_compile_definitions(${PROJECT_NAME}_lib
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(${PROJECT_NAME}_lib PUBLIC -Wall)

ament_target_dependencies(${PROJECT_NAME}_lib
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tmc_eigen_bridge
  tmc_vision_msgs
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_lib
  yaml-cpp
)

ament_environment_hooks(
  "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

add_executable(marker_based_localizer
  src/marker_based_localizer_main.cpp
)

target_include_directories(marker_based_localizer
  PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(marker_based_localizer
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(marker_based_localizer
  rclcpp
)

ament_export_dependencies(
  rclcpp
)

install(TARGETS
  ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS marker_based_localizer
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

  ament_add_gtest(marker_based_localizer_test
    test/marker_based_localizer-test_common.cpp
    test/marker_based_localizer-test_node.cpp
    test/marker_based_localizer-test.cpp
  )
  target_link_libraries(marker_based_localizer_test ${PROJECT_NAME}_lib)

  ament_add_gtest(marker_based_localizer_no_joint_list_test
    test/marker_based_localizer-test_common.cpp
    test/marker_based_localizer-test_node.cpp
    test/marker_based_localizer_no_joint_list-test.cpp
  )
  target_link_libraries(marker_based_localizer_no_joint_list_test ${PROJECT_NAME}_lib)

endif()
ament_package()
