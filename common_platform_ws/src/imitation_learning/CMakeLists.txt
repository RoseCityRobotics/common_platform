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

# Find ONNX Runtime
find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime
  PATHS 
    /usr/lib
    /usr/local/lib
    /opt/onnxruntime/lib
    ${CMAKE_SOURCE_DIR}/../onnxruntime/lib
  NO_DEFAULT_PATH
)

find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  PATHS
    /usr/include
    /usr/local/include
    /opt/onnxruntime/include
    ${CMAKE_SOURCE_DIR}/../onnxruntime/include
  NO_DEFAULT_PATH
)

if(ONNXRUNTIME_LIBRARY AND ONNXRUNTIME_INCLUDE_DIR)
  set(ONNXRUNTIME_FOUND TRUE)
  set(ONNXRUNTIME_LIBRARIES ${ONNXRUNTIME_LIBRARY})
  set(ONNXRUNTIME_INCLUDE_DIRS ${ONNXRUNTIME_INCLUDE_DIR})
  message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIBRARY}")
else()
  set(ONNXRUNTIME_FOUND FALSE)
  message(WARNING "ONNX Runtime not found - attempting to find via pkg-config")
  # Try pkg-config as fallback
  pkg_check_modules(ONNXRUNTIME onnxruntime)
  if(ONNXRUNTIME_FOUND)
    message(STATUS "Found ONNX Runtime via pkg-config")
  else()
    message(WARNING "ONNX Runtime not found - building without ONNX Runtime support")
  endif()
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

if(ONNXRUNTIME_FOUND)
  target_link_libraries(imitation_learning_node PUBLIC ${ONNXRUNTIME_LIBRARIES})
  target_include_directories(imitation_learning_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIRS})
  if(ONNXRUNTIME_CFLAGS_OTHER)
    target_compile_options(imitation_learning_node PRIVATE ${ONNXRUNTIME_CFLAGS_OTHER})
  endif()
  target_compile_definitions(imitation_learning_node PRIVATE HAVE_ONNXRUNTIME)
  message(STATUS "Building with ONNX Runtime support")
else()
  message(WARNING "ONNX Runtime not found - building without ONNX Runtime support")
  message(WARNING "Install ONNX Runtime: https://onnxruntime.ai/docs/install/")
  target_compile_definitions(imitation_learning_node PRIVATE NO_ONNXRUNTIME)
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

