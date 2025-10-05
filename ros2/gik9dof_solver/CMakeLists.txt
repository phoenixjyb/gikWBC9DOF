cmake_minimum_required(VERSION 3.10)
project(gik9dof_solver)

find_package(ament_cmake REQUIRED)
find_package(OpenMP REQUIRED)

set(GENERATED_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/generated")
set(GENERATED_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/gik9dof_solver/generated")
set(SHARED_ROBOTICS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/shared_robotics")
set(COLLISIONCODEGEN_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/collisioncodegen")
set(LIBCCD_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/libccd")

file(GLOB GENERATED_SOURCES CONFIGURE_DEPENDS
  "${GENERATED_SRC_DIR}/*.cpp"
  "${GENERATED_SRC_DIR}/*.c"
)

file(GLOB COLLISIONCODEGEN_SOURCES CONFIGURE_DEPENDS
  "${COLLISIONCODEGEN_SRC_DIR}/*.cpp"
)

file(GLOB LIBCCD_SOURCES CONFIGURE_DEPENDS
  "${LIBCCD_SRC_DIR}/*.c"
)

list(FILTER GENERATED_SOURCES EXCLUDE REGEX "_coder_.*")
list(FILTER GENERATED_SOURCES EXCLUDE REGEX "_mex")

set(WRAPPER_SOURCES
  src/solver_wrapper.cpp
)

add_library(${PROJECT_NAME}
  ${WRAPPER_SOURCES}
  ${GENERATED_SOURCES}
  ${COLLISIONCODEGEN_SOURCES}
  ${LIBCCD_SOURCES}
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${GENERATED_INC_DIR}>
    $<BUILD_INTERFACE:${SHARED_ROBOTICS_DIR}>
    $<BUILD_INTERFACE:${COLLISIONCODEGEN_SRC_DIR}>
    $<BUILD_INTERFACE:${LIBCCD_SRC_DIR}>
    $<BUILD_INTERFACE:${LIBCCD_SRC_DIR}/ccd>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/shared_robotics>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    OpenMP::OpenMP_CXX
)

if(TARGET OpenMP::OpenMP_C)
  target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_C)
endif()

if(GENERATED_SOURCES)
  target_compile_definitions(${PROJECT_NAME} PRIVATE GIK9DOF_CODEGEN_AVAILABLE)
endif()

ament_target_dependencies(${PROJECT_NAME})

install(DIRECTORY include/gik9dof_solver/ DESTINATION include/gik9dof_solver)
install(DIRECTORY ${SHARED_ROBOTICS_DIR}/ DESTINATION include/shared_robotics)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include include/shared_robotics)
ament_export_dependencies(OpenMP)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})

ament_package()
