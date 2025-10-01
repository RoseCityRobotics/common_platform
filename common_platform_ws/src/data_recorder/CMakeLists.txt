cmake_minimum_required(VERSION 3.8)
project(data_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Add executable
add_executable(data_recorder_node src/data_recorder_node.cpp)

# Specify libraries to link against using modern CMake
target_link_libraries(data_recorder_node PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  cv_bridge::cv_bridge
  ${OpenCV_LIBS}
  tf2::tf2
  tf2_ros::tf2_ros
)

# Install executable
install(TARGETS
  data_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(PROGRAMS
  scripts/process_training_data.py
  scripts/record_control.py
  scripts/example_usage.py
  scripts/test_libcamera.py
  scripts/test_namespace.py
  scripts/test_namespace_topics.py
  scripts/prepare_for_labelstudio.py
  scripts/combine_yolo_with_motor_data.py
  scripts/ramdisk_recorder.py
  scripts/ramdisk_control.py
  scripts/test_ramdisk.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
