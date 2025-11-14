cmake_minimum_required(VERSION 3.8)
project(imitation_learning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)

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

add_executable(imitation_learning_node src/imitation_learning_node.cpp)
target_include_directories(imitation_learning_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OPENCV_INCLUDE_DIRS})
target_compile_features(imitation_learning_node PUBLIC c_std_99 cxx_std_17)

# Link dependencies
target_link_libraries(imitation_learning_node PUBLIC
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
)

target_link_libraries(imitation_learning_node PUBLIC ${OPENCV_LIBRARIES})
target_compile_options(imitation_learning_node PRIVATE ${OPENCV_CFLAGS_OTHER})

if(HAILORT_FOUND)
  target_link_libraries(imitation_learning_node PUBLIC ${HAILORT_LIBRARIES})
  target_include_directories(imitation_learning_node PRIVATE ${HAILORT_INCLUDE_DIRS})
  target_compile_options(imitation_learning_node PRIVATE ${HAILORT_CFLAGS_OTHER})
  target_compile_definitions(imitation_learning_node PRIVATE HAVE_HAILORT)
else()
  message(WARNING "HailoRT not found - building without HailoRT support")
  target_compile_definitions(imitation_learning_node PRIVATE NO_HAILORT)
endif()

install(TARGETS imitation_learning_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_package()

