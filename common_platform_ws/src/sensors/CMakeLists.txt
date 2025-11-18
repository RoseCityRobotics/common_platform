cmake_minimum_required(VERSION 3.8)
project(sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(PROGRAMS
  scripts/calibrate_camera.py
  DESTINATION lib/${PROJECT_NAME}
)

# Build image flip node
# Note: Linker warnings about OpenCV version conflicts are harmless -
# we use OpenCV 4.6 to match cv_bridge, even though 4.12 is also installed
add_executable(image_flip_node src/image_flip_node.cpp)
target_include_directories(image_flip_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OPENCV_INCLUDE_DIRS})
target_compile_features(image_flip_node PUBLIC c_std_99 cxx_std_17)

# Link ROS2 dependencies using modern CMake targets
target_link_libraries(image_flip_node PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  cv_bridge::cv_bridge
  ${OPENCV_LIBRARIES}
)
target_compile_options(image_flip_node PRIVATE ${OPENCV_CFLAGS_OTHER})

install(TARGETS image_flip_node
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include
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
