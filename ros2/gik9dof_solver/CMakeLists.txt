cmake_minimum_required(VERSION 3.8)
project(gik9dof_solver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Enable OpenMP for parallel processing (ARM NEON)
find_package(OpenMP REQUIRED)

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gik9dof_msgs REQUIRED)

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/matlab_codegen/include  # MATLAB generated headers
)

# MATLAB Coder generated library
# After running MATLAB code generation, copy the generated files to:
#   - matlab_codegen/include/*.h
#   - matlab_codegen/lib/*.a (or *.so)
#
# Uncomment after code generation:
# add_library(gik_solver_lib STATIC IMPORTED)
# set_target_properties(gik_solver_lib PROPERTIES
#   IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/matlab_codegen/lib/libsolveGIKStepWrapper.a
# )

# Solver node executable
add_executable(gik9dof_solver_node
  src/gik9dof_solver_node.cpp
)

ament_target_dependencies(gik9dof_solver_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  gik9dof_msgs
)

target_link_libraries(gik9dof_solver_node
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
  # gik_solver_lib  # Uncomment after MATLAB code generation
)

# Install
install(TARGETS
  gik9dof_solver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
