cmake_minimum_required(VERSION 3.5)
project(tmc_velocity_acceleration_limiter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)


add_library(${PROJECT_NAME}_lib SHARED
  src/${PROJECT_NAME}/velocity_acceleration_limiter.cpp
)
target_compile_definitions(${PROJECT_NAME}_lib
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(${PROJECT_NAME}_lib PUBLIC -Wall)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  geometry_msgs
  std_srvs
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

add_executable(velocity_acceleration_limiter
  src/${PROJECT_NAME}/velocity_acceleration_limiter_main.cpp
)

target_include_directories(velocity_acceleration_limiter
  PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(velocity_acceleration_limiter
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(velocity_acceleration_limiter
  rclcpp
  geometry_msgs
  std_srvs
)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  std_srvs
)

install(TARGETS
  ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS velocity_acceleration_limiter
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
  find_package(ament_index_cpp REQUIRED)
  install(DIRECTORY test/parameter
    DESTINATION share/${PROJECT_NAME}/test
  )

  ament_add_gtest(velocity_acceleration_limiter_test
    test/velocity_acceleration_limiter-test.cpp
  )

  target_link_libraries(velocity_acceleration_limiter_test ${PROJECT_NAME}_lib)
  ament_target_dependencies(velocity_acceleration_limiter_test ament_index_cpp)

endif()
ament_package()
