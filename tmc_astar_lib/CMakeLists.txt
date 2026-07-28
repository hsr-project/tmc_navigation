cmake_minimum_required(VERSION 3.5)
project(tmc_astar_lib)

find_package(ament_cmake REQUIRED)
find_package(console_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tmc_pose_2d_lib REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/astar_executer.cpp
  src/astar_node.cpp
  src/astar_node_manager.cpp
  src/astar_queue.cpp
  src/layered_cost_map.cpp
  src/cost_corrector/direction_cost_corrector.cpp
  src/cost_corrector/dynamic_obstacle_cost_corrector.cpp
  src/cost_corrector/preferred_path_cost_corrector.cpp
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(${PROJECT_NAME} PUBLIC -Wall)
ament_target_dependencies(${PROJECT_NAME}
  console_bridge
  rclcpp
  tmc_pose_2d_lib
)
ament_export_dependencies(
  console_bridge
  rclcpp
  tmc_pose_2d_lib
)


target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # for astar_executer
  ament_add_gtest(astar_executer-test
    test/astar_executer-test.cpp)
  target_link_libraries(astar_executer-test ${PROJECT_NAME})
  # for astar_node_manager
  ament_add_gtest(astar_node_manager-test
    test/astar_node_manager-test.cpp)
  target_link_libraries(astar_node_manager-test ${PROJECT_NAME})
  # for astar_node
  ament_add_gtest(astar_node-test
    test/astar_node-test.cpp)
  target_link_libraries(astar_node-test ${PROJECT_NAME})
  # for astar_queue
  ament_add_gtest(astar_queue-test
    test/astar_queue-test.cpp)
  target_link_libraries(astar_queue-test ${PROJECT_NAME})
  # for layered_cost_map
  ament_add_gtest(layered_cost_map-test
    test/layered_cost_map-test.cpp)
  target_link_libraries(layered_cost_map-test ${PROJECT_NAME})
  # for direction_cost_corrector
  ament_add_gtest(direction_cost_corrector-test
    test/cost_corrector/direction_cost_corrector-test.cpp)
  target_link_libraries(direction_cost_corrector-test ${PROJECT_NAME})
  # for dynamic_obstacle_cost_corrector
  ament_add_gtest(dynamic_obstacle_cost_corrector-test
    test/cost_corrector/dynamic_obstacle_cost_corrector-test.cpp)
  target_link_libraries(dynamic_obstacle_cost_corrector-test ${PROJECT_NAME})
  # for preferred_path_cost_corrector
  ament_add_gtest(preferred_path_cost_corrector-test
    test/cost_corrector/preferred_path_cost_corrector-test.cpp)
  target_link_libraries(preferred_path_cost_corrector-test ${PROJECT_NAME})
endif()
ament_package()
