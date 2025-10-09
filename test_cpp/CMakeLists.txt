cmake_minimum_required(VERSION 3.15)
project(GIK_20Constraints_Test)

# C++14 required
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option to use ARM64 or x64 generated code
option(USE_ARM64_CODE "Use ARM64 generated code instead of x64" OFF)

# Find all generated source files
if(USE_ARM64_CODE)
    set(CODEGEN_DIR "${CMAKE_SOURCE_DIR}/../codegen/arm64_realtime")
    message(STATUS "Using ARM64 generated code (Linux) - MaxTime=50ms real-time version")
else()
    # Use archived x64 validation code for Windows testing
    set(CODEGEN_DIR "${CMAKE_SOURCE_DIR}/../codegen/archive/x64_wsl_validation_final")
    message(STATUS "Using x64 archived validation code (Windows)")
endif()

# Collect all .cpp and .c files from codegen directory (excluding interface and examples)
file(GLOB CODEGEN_SOURCES 
    "${CODEGEN_DIR}/*.cpp"
    "${CODEGEN_DIR}/*.c"
)

# Remove unwanted files
list(FILTER CODEGEN_SOURCES EXCLUDE REGEX ".*interface.*")
list(FILTER CODEGEN_SOURCES EXCLUDE REGEX ".*examples.*")
list(FILTER CODEGEN_SOURCES EXCLUDE REGEX ".*_mex\\.cpp")

# Include directories
include_directories(
    ${CODEGEN_DIR}
    ${CMAKE_SOURCE_DIR}
)

# Build test executable
add_executable(test_gik_20constraints
    test_gik_20constraints.cpp
    ${CODEGEN_SOURCES}
)

# Windows-specific settings
if(WIN32)
    target_compile_definitions(test_gik_20constraints PRIVATE _USE_MATH_DEFINES)
    # Suppress warnings from generated code
    target_compile_options(test_gik_20constraints PRIVATE /W3 /wd4244 /wd4267)
else()
    # Linux/WSL settings
    target_compile_options(test_gik_20constraints PRIVATE -Wall -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas)
    # Link math library, pthread, and OpenMP
    target_link_libraries(test_gik_20constraints m pthread gomp)
endif()

# Output directory
set_target_properties(test_gik_20constraints PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

message(STATUS "===========================================")
message(STATUS "GIK 20-Constraint Standalone Test")
message(STATUS "===========================================")
message(STATUS "Codegen directory: ${CODEGEN_DIR}")
message(STATUS "Source files found: ${CODEGEN_SOURCES}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "===========================================")
