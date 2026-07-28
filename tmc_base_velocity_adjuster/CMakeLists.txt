cmake_minimum_required(VERSION 3.5)
project(tmc_base_velocity_adjuster)

find_package(rclcpp REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)

add_executable(base_velocity_adjuster
  src/base_velocity_adjuster_main.cpp
)

target_include_directories(base_velocity_adjuster
  PUBLIC
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(base_velocity_adjuster
  rclcpp
  angles
  geometry_msgs
  pcl_ros
  sensor_msgs
  std_srvs
  tf2
  tf2_eigen
)

ament_export_dependencies(
  rclcpp
  angles
  geometry_msgs
  pcl_ros
  sensor_msgs
  std_srvs
  tf2
  tf2_eigen
)

install(TARGETS base_velocity_adjuster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  install(DIRECTORY test/parameter
    DESTINATION share/${PROJECT_NAME}/test
  )
  ament_add_gtest(base_velocity_adjuster_node-test test/base_velocity_adjuster_node-test.cpp)
  ament_target_dependencies(base_velocity_adjuster_node-test
    ament_index_cpp rclcpp angles geometry_msgs pcl_ros sensor_msgs std_srvs tf2 tf2_eigen)


  ament_add_gtest(base_velocity_optimizer-test test/base_velocity_optimizer-test.cpp)
  ament_target_dependencies(base_velocity_optimizer-test
    ament_index_cpp rclcpp angles geometry_msgs pcl_ros sensor_msgs std_srvs tf2 tf2_eigen)

  ament_add_gtest(collision_estimator-test test/collision_estimator-test.cpp)
  ament_target_dependencies(collision_estimator-test
    ament_index_cpp rclcpp angles geometry_msgs pcl_ros sensor_msgs std_srvs tf2 tf2_eigen)

  # 下記ros2/geometry2の課題により、テスト失敗することがあるため、テストを無効化する
  # https://github.com/ros2/geometry2/issues/517
  # TODO(kazuki_shibamiya) : テストを見直して、成功するようにする
  # ament_add_gtest(obstacle_converter-test test/obstacle_converter-test.cpp)
  # ament_target_dependencies(obstacle_converter-test
  #   rclcpp angles geometry_msgs pcl_ros sensor_msgs std_srvs tf2 tf2_eigen)

  ament_add_gtest(obstacle_input-test test/obstacle_input-test.cpp)
  ament_target_dependencies(obstacle_input-test
    ament_index_cpp rclcpp angles geometry_msgs pcl_ros sensor_msgs std_srvs tf2 tf2_eigen)
endif()

ament_package()
