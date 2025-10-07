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
  include/shared_robotics  # MATLAB Robotics System Toolbox collision headers
  include/velocity_controller  # MATLAB Coder generated velocity controller (simple heading)
  include/purepursuit  # MATLAB Coder generated Pure Pursuit controller
)

# MATLAB Coder generated source files
set(MATLAB_SOLVER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/matlab_codegen/include")
include_directories(${MATLAB_SOLVER_DIR})

# Collect all MATLAB generated source files
file(GLOB MATLAB_SOURCES 
  "${MATLAB_SOLVER_DIR}/*.cpp"
  "${MATLAB_SOLVER_DIR}/*.c"
)

# Create MATLAB solver library from source files
add_library(gik_matlab_solver STATIC ${MATLAB_SOURCES})
target_include_directories(gik_matlab_solver PUBLIC
  ${MATLAB_SOLVER_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/matlab_codegen/include
)

# Detect architecture and set appropriate flags
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  message(STATUS "Building for ARM64 architecture")
  # Disable x86 intrinsics on ARM
  target_compile_definitions(gik_matlab_solver PRIVATE 
    __SSE2__=0
    __AVX__=0
    __AVX2__=0
  )
  # Enable ARM NEON optimizations
  target_compile_options(gik_matlab_solver PRIVATE 
    -march=armv8-a
    -mtune=cortex-a78
  )
else()
  message(STATUS "Building for x86_64 architecture")
endif()

target_link_libraries(gik_matlab_solver
  OpenMP::OpenMP_CXX
)
# Disable warnings for MATLAB generated code
target_compile_options(gik_matlab_solver PRIVATE -w)

# ========================================
# Hybrid A* Planner (MATLAB Coder ARM64)
# ========================================
set(PLANNER_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/generated/planner")

# Collect all planner source files
file(GLOB PLANNER_SOURCES
    "${PLANNER_GENERATED_DIR}/*.cpp"
    "${PLANNER_GENERATED_DIR}/*.c"
)

# Remove main/example files if any (not needed for library)
list(FILTER PLANNER_SOURCES EXCLUDE REGEX ".*main\\.cpp$")
list(FILTER PLANNER_SOURCES EXCLUDE REGEX ".*example.*\\.cpp$")
list(FILTER PLANNER_SOURCES EXCLUDE REGEX ".*_rtw\\.cpp$")
list(FILTER PLANNER_SOURCES EXCLUDE REGEX ".*setup_.*\\.cpp$")

message(STATUS "Planner sources: ${PLANNER_SOURCES}")

# Create planner library
add_library(hybrid_astar_planner STATIC ${PLANNER_SOURCES})

target_include_directories(hybrid_astar_planner PUBLIC
  ${PLANNER_GENERATED_DIR}
)

# Apply same architecture-specific flags as GIK solver
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  message(STATUS "Configuring Hybrid A* planner for ARM64")
  target_compile_definitions(hybrid_astar_planner PRIVATE 
    __SSE2__=0
    __AVX__=0
    __AVX2__=0
  )
  target_compile_options(hybrid_astar_planner PRIVATE 
    -march=armv8-a
    -mtune=cortex-a78
  )
else()
  message(STATUS "Configuring Hybrid A* planner for x86_64")
endif()

# Disable warnings for MATLAB generated code
target_compile_options(hybrid_astar_planner PRIVATE -w)

# Solver node executable
add_executable(gik9dof_solver_node
  src/gik9dof_solver_node.cpp
  # src/stage_b_chassis_plan.cpp    # Stage B chassis planning controller (DISABLED - needs fixing)
  src/collisioncodegen_stubs.cpp  # Stub implementations for MATLAB collision functions
  src/coder_posix_time_stubs.cpp  # Stub implementations for MATLAB POSIX time functions
  # MATLAB Coder generated velocity controller (simple heading)
  src/velocity_controller/holisticVelocityController.cpp
  src/velocity_controller/holisticVelocityController_initialize.cpp
  src/velocity_controller/holisticVelocityController_terminate.cpp
  src/velocity_controller/wrapToPi.cpp
  # MATLAB Coder generated Pure Pursuit controller
  src/purepursuit/purePursuitVelocityController.cpp
  src/purepursuit/purePursuitVelocityController_initialize.cpp
  src/purepursuit/purePursuitVelocityController_terminate.cpp
  src/purepursuit/rtGetInf.cpp
  src/purepursuit/rtGetNaN.cpp
  src/purepursuit/rt_nonfinite.cpp
)

ament_target_dependencies(gik9dof_solver_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  gik9dof_msgs
)

target_link_libraries(gik9dof_solver_node
  gik_matlab_solver  # MATLAB generated solver library
  hybrid_astar_planner  # MATLAB generated planner library (NEW)
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
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
