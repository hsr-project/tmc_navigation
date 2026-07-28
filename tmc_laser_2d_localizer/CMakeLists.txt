cmake_minimum_required(VERSION 3.5)
project(tmc_laser_2d_localizer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tmc_navigation_msgs REQUIRED)
find_package(tmc_pose_2d_lib REQUIRED)
find_package(pcl_ros REQUIRED)


add_library(${PROJECT_NAME}_lib SHARED
  src/laser_2d_mcl_lib.c
  src/laser_2d_mcl.cpp)
target_compile_definitions(${PROJECT_NAME}_lib
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(${PROJECT_NAME}_lib PUBLIC -Wall)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  tmc_pose_2d_lib
)
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


add_executable(laser_2d_localizer
  src/laser_2d_localizer_main.cpp
  src/laser_2d_localizer_node.cpp
)

target_include_directories(laser_2d_localizer
  PUBLIC
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(laser_2d_localizer
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(laser_2d_localizer
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tmc_navigation_msgs
  tmc_pose_2d_lib
  pcl_ros
)

ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tmc_navigation_msgs
  tmc_pose_2d_lib
  pcl_ros
)

install(TARGETS
  ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS laser_2d_localizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Install test directory (for launch files, bags, etc.)
  install(DIRECTORY test/config
    DESTINATION share/${PROJECT_NAME}/test
  )
  install(DIRECTORY test/rosbag
    DESTINATION share/${PROJECT_NAME}/test
  )

  add_launch_test(
    test/laser_2d_localizer_node-test.py
    # Set a timeout longer than 
    # the rosbag playback duration (approx. 120 sec)
    TIMEOUT 150  
  )
  
endif()

ament_package()