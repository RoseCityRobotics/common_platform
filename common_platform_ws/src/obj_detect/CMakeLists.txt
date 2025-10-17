cmake_minimum_required(VERSION 3.8)
project(obj_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)  # Not needed for manual image conversion
find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)
find_package(rosidl_default_generators REQUIRED)

# Find HailoRT directly
find_library(HAILORT_LIBRARY
  NAMES hailort
  PATHS /usr/lib /usr/local/lib
  NO_DEFAULT_PATH
)

find_path(HAILORT_INCLUDE_DIR
  NAMES hailo/hailort.hpp
  PATHS /usr/include /usr/local/include
  NO_DEFAULT_PATH
)

if(HAILORT_LIBRARY AND HAILORT_INCLUDE_DIR)
  set(HAILORT_FOUND TRUE)
  set(HAILORT_LIBRARIES ${HAILORT_LIBRARY})
  set(HAILORT_INCLUDE_DIRS ${HAILORT_INCLUDE_DIR})
  message(STATUS "Found HailoRT: ${HAILORT_LIBRARY}")
else()
  set(HAILORT_FOUND FALSE)
  message(WARNING "HailoRT not found - building without HailoRT support")
endif()

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# Add message dependencies to the executable
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(object_detector src/object_detector.cpp)
target_include_directories(object_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OPENCV_INCLUDE_DIRS})
target_compile_features(object_detector PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Link dependencies
target_link_libraries(object_detector PUBLIC
      ${geometry_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      ${std_msgs_TARGETS}
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs_library
    )

target_link_libraries(object_detector PUBLIC ${OPENCV_LIBRARIES})
target_compile_options(object_detector PRIVATE ${OPENCV_CFLAGS_OTHER})

if(HAILORT_FOUND)
  target_link_libraries(object_detector PUBLIC ${HAILORT_LIBRARIES})
  target_include_directories(object_detector PRIVATE ${HAILORT_INCLUDE_DIRS})
  target_compile_options(object_detector PRIVATE ${HAILORT_CFLAGS_OTHER})
  target_compile_definitions(object_detector PRIVATE HAVE_HAILORT)
else()
  message(WARNING "HailoRT not found - building without HailoRT support")
  target_compile_definitions(object_detector PRIVATE NO_HAILORT)
endif()

install(TARGETS object_detector
  DESTINATION lib/${PROJECT_NAME})

# Install message interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(object_detector PUBLIC "${cpp_typesupport_target}")

# Install message files
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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
endif()

ament_package()
