cmake_minimum_required(VERSION 3.8)
project(gik9dof_controllers)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gik9dof_msgs REQUIRED)

# Include directories
include_directories(
  include
  src/matlab_generated/chassis_path_follower
)

# Collect all MATLAB generated source files
file(GLOB MATLAB_CHASSIS_SOURCES
  "src/matlab_generated/chassis_path_follower/*.cpp"
)

# Chassis Path Follower Node
add_executable(chassis_path_follower_node
  src/chassis_path_follower_node.cpp
  ${MATLAB_CHASSIS_SOURCES}
)

ament_target_dependencies(chassis_path_follower_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

# ARM64 specific optimizations (NEON SIMD)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  target_compile_options(chassis_path_follower_node PRIVATE
    -march=armv8-a
    -mtune=cortex-a78
    -O3
    -ffast-math
  )
  message(STATUS "Enabled ARM64 optimizations for chassis path follower")
endif()

# Install targets
install(TARGETS
  chassis_path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py" PATTERN "*.xml"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
