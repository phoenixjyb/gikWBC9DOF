cmake_minimum_required(VERSION 3.10)
project(gik9dof_controllers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gik9dof_solver REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COMMON_DEPENDENCIES
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  trajectory_msgs
  visualization_msgs
  std_srvs
)

set(CONTROLLER_SOURCES
  src/holistic_controller.cpp
  src/staged_controller.cpp
  src/trajectory_manager.cpp
  src/obstacle_provider.cpp
)

foreach(controller_src ${CONTROLLER_SOURCES})
  get_filename_component(target_name ${controller_src} NAME_WE)
  add_executable(${target_name} ${controller_src})
  target_include_directories(${target_name}
    PRIVATE
      include
      include/gik9dof_controllers
  )
  target_link_libraries(${target_name} gik9dof_solver::gik9dof_solver)
  ament_target_dependencies(${target_name} ${COMMON_DEPENDENCIES})
  install(TARGETS ${target_name}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(${COMMON_DEPENDENCIES} gik9dof_solver)
ament_package()
