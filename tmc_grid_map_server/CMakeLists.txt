cmake_minimum_required(VERSION 3.5)
project(tmc_grid_map_server)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tmc_navigation_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tmc_pose_2d_lib REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)

find_package(tf2_geometry_msgs)
find_package(tf2_ros)
find_package(yaml-cpp REQUIRED)



add_library(${PROJECT_NAME}_lib SHARED
  src/${PROJECT_NAME}/grid_map_server_node.cpp
  src/${PROJECT_NAME}/occupancy_grid_converter.cpp
  src/${PROJECT_NAME}/util.cpp
)
target_compile_definitions(${PROJECT_NAME}_lib
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(${PROJECT_NAME}_lib PUBLIC -Wall)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  nav_msgs
  tmc_navigation_msgs
  tf2
  tmc_pose_2d_lib
  tf2_geometry_msgs
)
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${SDL_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${SDL_IMAGE_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_lib
  yaml-cpp
  ${SDL_LIBRARY} ${SDL_IMAGE_LIBRARIES}
)

ament_environment_hooks(
  "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


add_executable(grid_map_server
  src/${PROJECT_NAME}/grid_map_server_main.cpp
)

target_include_directories(grid_map_server
  PUBLIC
    $<BUILD_INTERFACE:${SDL_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${SDL_IMAGE_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(grid_map_server
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(grid_map_server
  rclcpp
  nav_msgs
  tmc_navigation_msgs
  tf2
  tmc_pose_2d_lib
)

add_executable(occupancy_grid_converter_node
  src/${PROJECT_NAME}/occupancy_grid_converter_main.cpp
)

target_include_directories(occupancy_grid_converter_node
  PUBLIC
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(occupancy_grid_converter_node
  ${PROJECT_NAME}_lib
)
ament_target_dependencies(occupancy_grid_converter_node
  rclcpp
)

ament_export_dependencies(
  rclcpp
  nav_msgs
  tmc_navigation_msgs
  tf2
  tmc_pose_2d_lib
)

install(TARGETS
  ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS grid_map_server
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS occupancy_grid_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Boost REQUIRED)

  install(DIRECTORY test/maps
    DESTINATION share/${PROJECT_NAME}/test
  )
  install(DIRECTORY test/parameter
    DESTINATION share/${PROJECT_NAME}/test
  )

  ament_add_gtest(
    grid_map_server_square_test
    test/grid_map_server_node_test.cpp
    APPEND_ENV PARAMETER_FILE=grid_map_server_square_test.yaml
  )
  target_link_libraries(grid_map_server_square_test ${PROJECT_NAME}_lib)
  target_include_directories(grid_map_server_square_test PUBLIC ${Boost_INCLUDE_DIRS})

  ament_add_gtest(
    grid_map_server_vertical_test
    test/grid_map_server_node_test.cpp
    APPEND_ENV PARAMETER_FILE=grid_map_server_vertical_test.yaml
  )
  target_link_libraries(grid_map_server_vertical_test ${PROJECT_NAME}_lib)
  target_include_directories(grid_map_server_vertical_test PUBLIC ${Boost_INCLUDE_DIRS})

  ament_add_gtest(
    grid_map_server_horizontal_test
    test/grid_map_server_node_test.cpp
    APPEND_ENV PARAMETER_FILE=grid_map_server_horizontal_test.yaml
  )
  target_link_libraries(grid_map_server_horizontal_test ${PROJECT_NAME}_lib)
  target_include_directories(grid_map_server_horizontal_test PUBLIC ${Boost_INCLUDE_DIRS})
endif()

ament_package()
