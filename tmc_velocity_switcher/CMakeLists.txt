# Required CMake Version    
cmake_minimum_required(VERSION 3.5)
# Package Name
project(tmc_velocity_switcher)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/velocity_switcher.cpp
  src/input_velocity.cpp
)

target_compile_definitions(${PROJECT_NAME}_lib
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(${PROJECT_NAME}_lib PUBLIC -Wall)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  geometry_msgs
  std_msgs
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

add_executable(velocity_switcher
  src/velocity_switcher_main.cpp
)

target_include_directories(velocity_switcher
  PUBLIC
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(velocity_switcher
  ${PROJECT_NAME}_lib
)
ament_target_dependencies(velocity_switcher
  rclcpp
  rclcpp_components
  geometry_msgs
  std_msgs
)

ament_export_dependencies(
  rclcpp
  rclcpp_components
  geometry_msgs
  std_msgs
)


install(TARGETS
  ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS velocity_switcher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  install(DIRECTORY test/parameter
    DESTINATION share/${PROJECT_NAME}/test
  )

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  ament_add_gtest(velocity_switcher-test
    test/velocity_switcher-test.cpp
    TIMEOUT 120)
  target_link_libraries(velocity_switcher-test ${PROJECT_NAME}_lib)
  ament_target_dependencies(velocity_switcher-test ament_index_cpp)
endif()
ament_package()
