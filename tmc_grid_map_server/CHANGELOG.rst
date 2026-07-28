^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tmc_grid_map_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2026-06-29)
-------------------
* Migration to ROS2 jazzy
* Contributors: Kazuki Shibamiya, Hozumi Inoue, Katsushi Fukuoka, Yoshimi Iyoda

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tmc_grid_map_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2025-12-04)
-------------------
* Except unstable tests.
* マージ先 ros2/feature/tmc_marker_based_localizer-for-humble
* Fix build dependencies.
* Porting tmc_marker_based_localizer to ROS2 humble and add test.
* Fix flake8 and cpplint error.
* Fix build and test dependencies.
* Horizontal deployment of fix that cause malloc errors at the end of test.
* Fix indentation for subscriber initialization in ObstacleInput constructor
* fix indent of velocity_switcher-test.cpp
* Fix indentation in main function for better readability
* Fix parameter declaration in SetFilterLeafSizeParam and SetFilterAreaRadiusParam methods
* Remove unnecessary blank line in main function of base_velocity_adjuster_node-test
* Fix indentation for subscriber initialization in ObstacleInput constructor
* Fix parameter declaration for filter_area_radius in ObstacleInputTest
* rebase obstacle_converter-test.cpp
* Organize code to set QoS profile to BEST_EFFORT
* Set QoS profile to BEST_EFFORT and updated obstacle topic subscriptions
* Fixed join without detaching thread in main function of test and added rclcpp::shutdown()
* Add dependency of package tf2_eigen to CMakeLists.txt and modify parameter declarations in tests and succeed in colocon build
* Fixed to specify type when declaring parameters
* Adopt to hubmle
* ros2 humble compatibility for the tmc_safety_velocity_limiter package
* Fix tmc_move_base test dependencies.
* Fix tmc_grid_map_server map publish rate.
* Fix tmc_map_merger parameter.
* [misc] Use double quotation marks throughout the code for consistency
* [update] Add '-l' option in rosbag play and processes to check the loop
* [fix] Fix formatting of nanosec in logging in the test
* [fix] Fix syntax error in 'CMakeLists.txt'
* [fix] Fix timestamp for looking up TF in 'src/laser_2d_localizer_node.cpp'
* [update] Add timeout in the launch test
* [update] Read initial values from 'laser_2d_localizer' via a service call
* [update] Add dependency on 'tf_transformations'
* [update] Change impl of 'get_yaw', change names of the constant variables
* [misc] Fix the unintentionally formated file 'src/laser_2d_localizer_node.cpp'  back to the original
* [misc] Remove unnecessary comment in 'CMakeLists.txt'
* [fix] Fix possible format inconsistency
* [misc] Add comment
* [refactor] Remove unnecessary commented-out lines
* [fix] Fix the problem of test failure due to mismatch between estimated pose and GT TF
* Implement launch_test to test 'tmc_laser_2d_localizer' with a bag file
* Porting tmc_imu_wheel_odometry test to ROS2 humble.
* Refactor test main function to ensure proper shutdown and thread management
* Fixed to specify type when declaring parameters
* Horizontal deployment of fix that cause malloc errors at the end of test.
* refactor : indent width
* update test function
* Porting tmc_point_cloud_accumulator test to ROS2 humble.
* Update tmc_map_merger package for Humble support
* change range-based for loop to use const reference
* replace double variable with bool to reflect intended usage
* refactor private method names by deleting "_" suffix
* add action server/client and path topic callback
* migrate from boost::optional and boost::random to std libraries
* make minimal changes to enable build
* Porting tmc_viewpoint_controller to ROS2 humble.
* Porting tmc_pose_integrator test to ROS2 humble.
* Porting tmc_velocity_acceleration_limiter test to ROS2 humble.
* fix tf2 eigen bug
* Contributors: Hiroaki Kuroda, Hiromichi Nakashima, Kosei Tanada, Shigemichi Matsuzaki, ShigemichiMatsuzaki, Takuma Sugino, Yu Hirota, shintaro nakaoka, takuma_sugino, 柴宮 和希

2.0.0 (2025-05-13)
-------------------
* Migration to ROS2 humble
* Contributors: Keisuke Takeshita, Syuuhei Shiro