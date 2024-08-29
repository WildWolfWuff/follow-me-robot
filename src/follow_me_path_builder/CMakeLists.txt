cmake_minimum_required(VERSION 3.8)
project(follow_me_path_builder)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(follow_me_path_builder
  src/node_main.cpp
  src/path_builder.cpp
)

target_include_directories(
  follow_me_path_builder
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  follow_me_path_builder
  rclcpp
  tf2_ros
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  follow_me_path_builder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

# ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name camera_publisher follow_me_camera_pub --dependencies rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV