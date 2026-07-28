cmake_minimum_required(VERSION 3.5)
project(tmc_map_merger)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/map_merger_node.cpp
  src/map_merger_factory.cpp
  src/map_input_factory.cpp
  src/point_cloud_filter_factory.cpp)

target_compile_definitions(${PROJECT_NAME}_lib
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(${PROJECT_NAME}_lib PUBLIC -Wall)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  geometry_msgs
  nav_msgs
  pcl_ros
  std_srvs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


add_executable(map_merger src/map_merger_main.cpp)

target_include_directories(map_merger
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(map_merger
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(map_merger
  rclcpp
  geometry_msgs
  nav_msgs
  pcl_ros
  std_srvs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  pcl_ros
  std_srvs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS map_merger
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  install(DIRECTORY test/parameter
    DESTINATION share/${PROJECT_NAME}/test
  )

  ament_add_gtest(map-test test/map-test.cpp)
  target_link_libraries(map-test ${PROJECT_NAME}_lib)

  ament_add_gtest(map_drawer-test test/map_drawer-test.cpp)
  target_link_libraries(map_drawer-test ${PROJECT_NAME}_lib)

  ament_add_gtest(temporal_memories-test test/temporal_memories-test.cpp)
  target_link_libraries(temporal_memories-test ${PROJECT_NAME}_lib)

  ament_add_gtest(point_cloud_filter-test test/point_cloud_filter-test.cpp)
  target_link_libraries(point_cloud_filter-test ${PROJECT_NAME}_lib)
  ament_target_dependencies(point_cloud_filter-test ament_index_cpp)

  ament_add_gtest(map_input-test test/map_input-test.cpp)
  target_link_libraries(map_input-test ${PROJECT_NAME}_lib)
  ament_target_dependencies(map_input-test ament_index_cpp)

  ament_add_gtest(param-test test/param-test.cpp)
  target_link_libraries(param-test ${PROJECT_NAME}_lib)
  ament_target_dependencies(param-test ament_index_cpp)

  ament_add_gtest(map_converter-test test/map_converter-test.cpp)
  target_link_libraries(map_converter-test ${PROJECT_NAME}_lib)
  ament_target_dependencies(map_converter-test ament_index_cpp)

  ament_add_gtest(map_merger_node-test test/map_merger_node-test.cpp)
  target_link_libraries(map_merger_node-test ${PROJECT_NAME}_lib)
  ament_target_dependencies(map_merger_node-test ament_index_cpp)
endif()
ament_package()
