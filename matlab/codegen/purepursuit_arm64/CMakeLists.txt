###########################################################################
# CMakeLists.txt generated for component purePursuitVelocityController
# Product type: STATIC library
###########################################################################
cmake_minimum_required(VERSION 3.12)
project(purePursuitVelocityController)

# Propagate the CMAKE_EXPORT_COMPILE_COMMANDS variable from the 
# environment if it is defined as an environment variable, but not as a 
# CMake variable. This is to work around a bug in CMake 3.19 when the 
# "NMake Makefiles" generator is selected.
if(DEFINED ENV{CMAKE_EXPORT_COMPILE_COMMANDS} AND NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS $ENV{CMAKE_EXPORT_COMPILE_COMMANDS})
endif()


###########################################################################
## Path variables
###########################################################################
# Derive an absolute path to the code generation anchor folder.
get_filename_component(START_DIR . ABSOLUTE)

# Special directories defined by using CACHE variables can be overridden 
# by setting the variable from the command line, e.g.,
# 
# 	cmake . -DMATLAB_ROOT=/path/to/another/matlab/root
set(MATLAB_ROOT D:/Program\ Files/MATLAB/R2024b CACHE PATH "")

# Additional variables that are defined conditionally.
if("${CMAKE_CURRENT_BINARY_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
    set(BINARY_START_DIR "${START_DIR}")
else()
    set(BINARY_START_DIR "${CMAKE_BINARY_DIR}")
endif()

###########################################################################
## System Libraries
###########################################################################
find_library(FOUND_LIBM m NO_SYSTEM_ENVIRONMENT_PATH PATHS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES} ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})

###########################################################################
## Target definition and commands
###########################################################################

# Definition of target "purePursuitVelocityController".
add_library(purePursuitVelocityController STATIC ${START_DIR}/rt_nonfinite.cpp
    ${START_DIR}/rtGetNaN.cpp
    ${START_DIR}/rtGetInf.cpp
    ${START_DIR}/purePursuitVelocityController_initialize.cpp
    ${START_DIR}/purePursuitVelocityController_terminate.cpp
    ${START_DIR}/purePursuitVelocityController.cpp)

# Set properties for target "purePursuitVelocityController".
set_target_properties(purePursuitVelocityController PROPERTIES PREFIX ""
    POSITION_INDEPENDENT_CODE ON
    RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/./$<0:>"
    LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/./$<0:>"
    ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/./$<0:>")

# Specify language features required for target 
# "purePursuitVelocityController".
target_compile_features(purePursuitVelocityController PUBLIC cxx_std_11)

# Specify compiler preprocessor definitions for target 
# "purePursuitVelocityController".
target_compile_definitions(purePursuitVelocityController PRIVATE -DMODEL=purePursuitVelocityController)

# Specify include directories for target "purePursuitVelocityController".
target_include_directories(purePursuitVelocityController PUBLIC $<BUILD_INTERFACE:${START_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
    H:/wSpace/codegenGIKsample/Trial/gikWBC9DOF/matlab
    $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>)

# Specify library link dependencies for target 
# "purePursuitVelocityController". CMake generator expressions are used to 
# create a CMakeLists.txt file that supports multiple platforms with 
# differently named system library dependencies.
target_link_libraries(purePursuitVelocityController PUBLIC $<$<BOOL:${FOUND_LIBM}>:m>)


###########################################################################
## Build success message
###########################################################################
add_custom_command(TARGET purePursuitVelocityController POST_BUILD
     COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "\\#\\#\\# Created STATIC library: $<TARGET_FILE:purePursuitVelocityController>")

###########################################################################
## Call toolchain hook function if defined
###########################################################################
if(COMMAND toolchain_target_hook)
    toolchain_target_hook(purePursuitVelocityController)
endif()
