cmake_minimum_required(VERSION 3.8)
project(my_robot_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =======================
# Find dependencies
# =======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# =======================
# Build executable
# =======================
add_executable(lidar_listener src/lidar_listener.cpp)

ament_target_dependencies(lidar_listener
  rclcpp
  sensor_msgs
)

# =======================
# Install target
# =======================
install(TARGETS
  lidar_listener
  DESTINATION lib/${PROJECT_NAME}
)

# =======================
# Testing
# =======================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =======================
# MUST BE LAST
# =======================
ament_package()

