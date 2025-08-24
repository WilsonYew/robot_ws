cmake_minimum_required(VERSION 3.8)
project(stereo_camera_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- Executables ---
add_executable(stereo_calibration_capture src/stereo_calibration_capture.cpp)
ament_target_dependencies(stereo_calibration_capture rclcpp cv_bridge OpenCV)

# Only compile stereo_camera_node.cpp now
add_executable(stereo_camera_node src/stereo_camera_node.cpp)
ament_target_dependencies(stereo_camera_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(stereo_camera_node yaml-cpp ${OpenCV_LIBS})

# --- Install ---
install(TARGETS
  stereo_camera_node
  stereo_calibration_capture
  DESTINATION lib/${PROJECT_NAME})

ament_package()
