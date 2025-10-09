###########################################################################
## Makefile generated for component 'solveGIKStepWrapper'. 
## 
## Makefile     : solveGIKStepWrapper_rtw.mk
## Generated on : Thu Oct 09 09:42:02 2025
## Final product: ./solveGIKStepWrapper.a
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# MODELLIB                Static library target

PRODUCT_NAME              = solveGIKStepWrapper
MAKEFILE                  = solveGIKStepWrapper_rtw.mk
MATLAB_ROOT               = /home/yanbo/MATLAB/R2024a
MATLAB_BIN                = /home/yanbo/MATLAB/R2024a/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/glnxa64
START_DIR                 = /mnt/h/wSpace/codegenGIKsample/Trial/gikWBC9DOF/codegen/x86_64_validation_noCollision
TGT_FCN_LIB               = ISO_C++11
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = .
C_STANDARD_OPTS           = -fwrapv
CPP_STANDARD_OPTS         = -fwrapv
MODELLIB                  = solveGIKStepWrapper.a

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          GNU gcc/g++ | gmake (64-bit Linux)
# Supported Version(s):    
# ToolchainInfo Version:   2024a
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# C_STANDARD_OPTS
# CPP_STANDARD_OPTS

#-----------
# MACROS
#-----------

WARN_FLAGS         = -Wall -W -Wwrite-strings -Winline -Wstrict-prototypes -Wnested-externs -Wpointer-arith -Wcast-align -Wno-stringop-overflow
WARN_FLAGS_MAX     = $(WARN_FLAGS) -Wcast-qual -Wshadow
CPP_WARN_FLAGS     = -Wall -W -Wwrite-strings -Winline -Wpointer-arith -Wcast-align -Wno-stringop-overflow
CPP_WARN_FLAGS_MAX = $(CPP_WARN_FLAGS) -Wcast-qual -Wshadow

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = 

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: GNU C Compiler
CC = gcc

# Linker: GNU Linker
LD = g++

# C++ Compiler: GNU C++ Compiler
CPP = g++

# C++ Linker: GNU C++ Linker
CPP_LD = g++

# Archiver: GNU Archiver
AR = ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GMAKE Utility
MAKE_PATH = %MATLAB%/bin/glnxa64
MAKE = "$(MAKE_PATH)/gmake"


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  = @rm -f
ECHO                = @echo
MV                  = @mv
RUN                 =

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = ruvs
CFLAGS               = -c $(C_STANDARD_OPTS) -fPIC \
                       -O3
CPPFLAGS             = -c $(CPP_STANDARD_OPTS) -fPIC \
                       -O3
CPP_LDFLAGS          =
CPP_SHAREDLIB_LDFLAGS  = -shared -Wl,--no-undefined
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              =
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -shared -Wl,--no-undefined



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = ./solveGIKStepWrapper.a
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I/mnt/h/wSpace/codegenGIKsample/Trial/gikWBC9DOF -I$(MATLAB_ROOT)/extern/include/shared_robotics -I$(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src -I$(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd -I$(MATLAB_ROOT)/extern/include

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -Dccd_EXPORTS
DEFINES_CUSTOM = 
DEFINES_STANDARD = -DMODEL=solveGIKStepWrapper

DEFINES = $(DEFINES_) $(DEFINES_CUSTOM) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_ccd.c $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_mpr.c $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_polytope.c $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_vec3.c $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_CollisionGeometry.cpp $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_api.cpp $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_ccdExtensions.cpp $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_checkCollision.cpp $(START_DIR)/coder_posix_time.c $(START_DIR)/solveGIKStepWrapper_data.cpp $(START_DIR)/rt_nonfinite.cpp $(START_DIR)/rtGetNaN.cpp $(START_DIR)/rtGetInf.cpp $(START_DIR)/buildRobotForCodegen.cpp $(START_DIR)/rand.cpp $(START_DIR)/xzlangeM.cpp $(START_DIR)/xnrm2.cpp $(START_DIR)/xdotc.cpp $(START_DIR)/validatestring.cpp $(START_DIR)/diff.cpp $(START_DIR)/svd.cpp $(START_DIR)/eye.cpp $(START_DIR)/find.cpp $(START_DIR)/ixfun.cpp $(START_DIR)/tic.cpp $(START_DIR)/norm.cpp $(START_DIR)/toc.cpp $(START_DIR)/randn.cpp $(START_DIR)/ismember.cpp $(START_DIR)/GIKSolver.cpp $(START_DIR)/eml_rand_mt19937ar_stateful.cpp $(START_DIR)/strcmp.cpp $(START_DIR)/eml_rand_mt19937ar.cpp $(START_DIR)/xaxpy.cpp $(START_DIR)/xrotg.cpp $(START_DIR)/sqrt.cpp $(START_DIR)/xrot.cpp $(START_DIR)/xswap.cpp $(START_DIR)/mldivide.cpp $(START_DIR)/xgeqp3.cpp $(START_DIR)/sort.cpp $(START_DIR)/sortIdx.cpp $(START_DIR)/xzlascl.cpp $(START_DIR)/GIKProblem.cpp $(START_DIR)/ErrorDampedLevenbergMarquardt.cpp $(START_DIR)/rigidBodyJoint.cpp $(START_DIR)/structConstructorHelper.cpp $(START_DIR)/KinematicConstraint.cpp $(START_DIR)/RigidBodyTree.cpp $(START_DIR)/CharacterVector.cpp $(START_DIR)/CollisionGeometry.cpp $(START_DIR)/string1.cpp $(START_DIR)/CollisionSet.cpp $(START_DIR)/RigidBody.cpp $(START_DIR)/constraintDistanceBounds.cpp $(START_DIR)/FastVisualizationHelper.cpp $(START_DIR)/SystemTimeProvider.cpp $(START_DIR)/constraintPoseTarget.cpp $(START_DIR)/constraintJointBounds.cpp $(START_DIR)/rigidBody1.cpp $(START_DIR)/rigidBodyTree1.cpp $(START_DIR)/generalizedInverseKinematics.cpp $(START_DIR)/PoseTarget.cpp $(START_DIR)/JointPositionBounds.cpp $(START_DIR)/DistanceBoundsConstraint.cpp $(START_DIR)/CoderTimeAPI.cpp $(START_DIR)/solveGIKStepWrapper_rtwutil.cpp

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = ccd_ccd.o ccd_mpr.o ccd_polytope.o ccd_vec3.o collisioncodegen_CollisionGeometry.o collisioncodegen_api.o collisioncodegen_ccdExtensions.o collisioncodegen_checkCollision.o coder_posix_time.o solveGIKStepWrapper_data.o rt_nonfinite.o rtGetNaN.o rtGetInf.o buildRobotForCodegen.o rand.o xzlangeM.o xnrm2.o xdotc.o validatestring.o diff.o svd.o eye.o find.o ixfun.o tic.o norm.o toc.o randn.o ismember.o GIKSolver.o eml_rand_mt19937ar_stateful.o strcmp.o eml_rand_mt19937ar.o xaxpy.o xrotg.o sqrt.o xrot.o xswap.o mldivide.o xgeqp3.o sort.o sortIdx.o xzlascl.o GIKProblem.o ErrorDampedLevenbergMarquardt.o rigidBodyJoint.o structConstructorHelper.o KinematicConstraint.o RigidBodyTree.o CharacterVector.o CollisionGeometry.o string1.o CollisionSet.o RigidBody.o constraintDistanceBounds.o FastVisualizationHelper.o SystemTimeProvider.o constraintPoseTarget.o constraintJointBounds.o rigidBody1.o rigidBodyTree1.o generalizedInverseKinematics.o PoseTarget.o JointPositionBounds.o DistanceBoundsConstraint.o CoderTimeAPI.o solveGIKStepWrapper_rtwutil.o

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS =  -L"$(MATLAB_ROOT)/sys/os/glnxa64" -lm -liomp5

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_OPTS = -fopenmp
CFLAGS_TFL = -msse2 -fno-predictive-commoning
CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_OPTS) $(CFLAGS_TFL) $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_OPTS = -fopenmp
CPPFLAGS_TFL = -msse2 -fno-predictive-commoning
CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_OPTS) $(CPPFLAGS_TFL) $(CPPFLAGS_BASIC)

###########################################################################
## INLINED COMMANDS
###########################################################################

###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	@echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) $(OBJS)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /mnt/h/wSpace/codegenGIKsample/Trial/gikWBC9DOF/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : /mnt/h/wSpace/codegenGIKsample/Trial/gikWBC9DOF/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /mnt/h/wSpace/codegenGIKsample/Trial/gikWBC9DOF/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /mnt/h/wSpace/codegenGIKsample/Trial/gikWBC9DOF/%.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /mnt/h/wSpace/codegenGIKsample/Trial/gikWBC9DOF/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /mnt/h/wSpace/codegenGIKsample/Trial/gikWBC9DOF/%.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /mnt/h/wSpace/codegenGIKsample/Trial/gikWBC9DOF/%.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/%.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/%.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/%.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/%.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/%.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/%.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


ccd_ccd.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_ccd.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ccd_mpr.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_mpr.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ccd_polytope.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_polytope.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ccd_vec3.o : $(MATLAB_ROOT)/toolbox/shared/robotics/externalDependency/libccd/src/ccd_vec3.c
	$(CC) $(CFLAGS) -o "$@" "$<"


collisioncodegen_CollisionGeometry.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_CollisionGeometry.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


collisioncodegen_api.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_api.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


collisioncodegen_ccdExtensions.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_ccdExtensions.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


collisioncodegen_checkCollision.o : $(MATLAB_ROOT)/toolbox/shared/robotics/robotcore/builtins/libsrc/collisioncodegen/collisioncodegen_checkCollision.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


coder_posix_time.o : $(START_DIR)/coder_posix_time.c
	$(CC) $(CFLAGS) -o "$@" "$<"


solveGIKStepWrapper_data.o : $(START_DIR)/solveGIKStepWrapper_data.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rt_nonfinite.o : $(START_DIR)/rt_nonfinite.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rtGetNaN.o : $(START_DIR)/rtGetNaN.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rtGetInf.o : $(START_DIR)/rtGetInf.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


buildRobotForCodegen.o : $(START_DIR)/buildRobotForCodegen.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rand.o : $(START_DIR)/rand.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xzlangeM.o : $(START_DIR)/xzlangeM.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xnrm2.o : $(START_DIR)/xnrm2.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xdotc.o : $(START_DIR)/xdotc.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


validatestring.o : $(START_DIR)/validatestring.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


diff.o : $(START_DIR)/diff.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


svd.o : $(START_DIR)/svd.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


eye.o : $(START_DIR)/eye.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


find.o : $(START_DIR)/find.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


ixfun.o : $(START_DIR)/ixfun.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


tic.o : $(START_DIR)/tic.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


norm.o : $(START_DIR)/norm.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


toc.o : $(START_DIR)/toc.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


randn.o : $(START_DIR)/randn.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


ismember.o : $(START_DIR)/ismember.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


GIKSolver.o : $(START_DIR)/GIKSolver.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


eml_rand_mt19937ar_stateful.o : $(START_DIR)/eml_rand_mt19937ar_stateful.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


strcmp.o : $(START_DIR)/strcmp.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


eml_rand_mt19937ar.o : $(START_DIR)/eml_rand_mt19937ar.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xaxpy.o : $(START_DIR)/xaxpy.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xrotg.o : $(START_DIR)/xrotg.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sqrt.o : $(START_DIR)/sqrt.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xrot.o : $(START_DIR)/xrot.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xswap.o : $(START_DIR)/xswap.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


mldivide.o : $(START_DIR)/mldivide.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xgeqp3.o : $(START_DIR)/xgeqp3.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sort.o : $(START_DIR)/sort.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sortIdx.o : $(START_DIR)/sortIdx.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xzlascl.o : $(START_DIR)/xzlascl.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


GIKProblem.o : $(START_DIR)/GIKProblem.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


ErrorDampedLevenbergMarquardt.o : $(START_DIR)/ErrorDampedLevenbergMarquardt.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rigidBodyJoint.o : $(START_DIR)/rigidBodyJoint.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


structConstructorHelper.o : $(START_DIR)/structConstructorHelper.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


KinematicConstraint.o : $(START_DIR)/KinematicConstraint.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


RigidBodyTree.o : $(START_DIR)/RigidBodyTree.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


CharacterVector.o : $(START_DIR)/CharacterVector.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


CollisionGeometry.o : $(START_DIR)/CollisionGeometry.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


string1.o : $(START_DIR)/string1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


CollisionSet.o : $(START_DIR)/CollisionSet.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


RigidBody.o : $(START_DIR)/RigidBody.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


constraintDistanceBounds.o : $(START_DIR)/constraintDistanceBounds.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


FastVisualizationHelper.o : $(START_DIR)/FastVisualizationHelper.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


SystemTimeProvider.o : $(START_DIR)/SystemTimeProvider.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


constraintPoseTarget.o : $(START_DIR)/constraintPoseTarget.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


constraintJointBounds.o : $(START_DIR)/constraintJointBounds.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rigidBody1.o : $(START_DIR)/rigidBody1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rigidBodyTree1.o : $(START_DIR)/rigidBodyTree1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


generalizedInverseKinematics.o : $(START_DIR)/generalizedInverseKinematics.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


PoseTarget.o : $(START_DIR)/PoseTarget.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


JointPositionBounds.o : $(START_DIR)/JointPositionBounds.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


DistanceBoundsConstraint.o : $(START_DIR)/DistanceBoundsConstraint.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


CoderTimeAPI.o : $(START_DIR)/CoderTimeAPI.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


solveGIKStepWrapper_rtwutil.o : $(START_DIR)/solveGIKStepWrapper_rtwutil.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "### PRODUCT = $(PRODUCT)"
	@echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "### BUILD_TYPE = $(BUILD_TYPE)"
	@echo "### INCLUDES = $(INCLUDES)"
	@echo "### DEFINES = $(DEFINES)"
	@echo "### ALL_SRCS = $(ALL_SRCS)"
	@echo "### ALL_OBJS = $(ALL_OBJS)"
	@echo "### LIBS = $(LIBS)"
	@echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "### CFLAGS = $(CFLAGS)"
	@echo "### LDFLAGS = $(LDFLAGS)"
	@echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "### CPPFLAGS = $(CPPFLAGS)"
	@echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "### ARFLAGS = $(ARFLAGS)"
	@echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files ..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(ECHO) "### Deleted all derived files."


