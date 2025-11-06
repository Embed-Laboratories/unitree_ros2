cmake_minimum_required(VERSION 3.5)
project(unitree_ros2_example)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)

# Build go2_camera_gstreamer executable
add_executable(go2_camera_gstreamer src/go2/go2_camera_gstreamer.cpp)
target_compile_features(go2_camera_gstreamer PRIVATE cxx_std_14)
target_link_libraries(go2_camera_gstreamer ${OpenCV_LIBS} CURL::libcurl)

ament_target_dependencies(go2_camera_gstreamer
  rclcpp
  sensor_msgs
  cv_bridge
)

# Install
install(TARGETS go2_camera_gstreamer
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
