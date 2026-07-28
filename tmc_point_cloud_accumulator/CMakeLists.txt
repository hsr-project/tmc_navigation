cmake_minimum_required(VERSION 3.5)
project(tmc_point_cloud_accumulator)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# point_cloud_accumulator
add_library(${PROJECT_NAME}_point_cloud_accumulator SHARED
  src/${PROJECT_NAME}/point_cloud_accumulator.cpp
  src/${PROJECT_NAME}/point_cloud_accumulator_node.cpp
)
target_compile_definitions(${PROJECT_NAME}_point_cloud_accumulator
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(${PROJECT_NAME}_point_cloud_accumulator PUBLIC -Wall)
ament_target_dependencies(${PROJECT_NAME}_point_cloud_accumulator
  rclcpp
  geometry_msgs
  pcl_ros
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
)
target_include_directories(${PROJECT_NAME}_point_cloud_accumulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)



# point_cloud_merger
add_library(${PROJECT_NAME}_point_cloud_merger SHARED
  src/${PROJECT_NAME}/point_cloud_merger_node.cpp
)
target_compile_definitions(${PROJECT_NAME}_point_cloud_merger
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(${PROJECT_NAME}_point_cloud_merger PUBLIC -Wall)

ament_target_dependencies(${PROJECT_NAME}_point_cloud_merger
  rclcpp
  geometry_msgs
  pcl_ros
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
)
target_include_directories(${PROJECT_NAME}_point_cloud_merger
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)




# point_cloud_accumulator_node
add_executable(${PROJECT_NAME}_point_cloud_accumulator_node
  src/${PROJECT_NAME}/point_cloud_accumulator_main.cpp
)
target_include_directories(${PROJECT_NAME}_point_cloud_accumulator_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_point_cloud_accumulator_node
  ${PROJECT_NAME}_point_cloud_accumulator
)
set_target_properties(
  ${PROJECT_NAME}_point_cloud_accumulator_node
  PROPERTIES OUTPUT_NAME point_cloud_accumulator_node
)


# point_cloud_merger_node
add_executable(${PROJECT_NAME}_point_cloud_merger_node
  src/${PROJECT_NAME}/point_cloud_merger_main.cpp
)
target_include_directories(${PROJECT_NAME}_point_cloud_merger_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_point_cloud_merger_node
  ${PROJECT_NAME}_point_cloud_merger
)
set_target_properties(
  ${PROJECT_NAME}_point_cloud_merger_node
  PROPERTIES OUTPUT_NAME point_cloud_merger_node
)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  pcl_ros
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

# install
install(TARGETS
  ${PROJECT_NAME}_point_cloud_accumulator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${PROJECT_NAME}_point_cloud_merger
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_point_cloud_accumulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_point_cloud_merger_node
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
  find_package(ament_cmake_python REQUIRED)
  find_package(rclpy REQUIRED)
  ament_python_install_package("test")

  find_package(launch_testing_ament_cmake REQUIRED)

  install(DIRECTORY test/rosbag
    DESTINATION share/${PROJECT_NAME}/test
  )
  install(DIRECTORY test/parameter
    DESTINATION share/${PROJECT_NAME}/test
  )

  add_launch_test(test/point_cloud_accumulator_test.py TIMEOUT 60)
  # 下記未対応のため、use_cloud_1,2を切り替えるテストは未対応。
  # TODO(kazuki_shibamiya): use_cloud_1,2は現状true固定。パラメータ公開し、変更をコールバックで受け取れるようにする
  add_launch_test(test/point_cloud_merger_test_all_cloud_inside_range.py TIMEOUT 30)
  add_launch_test(test/point_cloud_merger_test_cloud1_outside_range.py TIMEOUT 30)
  add_launch_test(test/point_cloud_merger_test_cloud2_one_point_outside_range.py TIMEOUT 30)
  add_launch_test(test/point_cloud_merger_test_cloud2_two_point_outside_range.py TIMEOUT 30)
endif()

ament_package()
