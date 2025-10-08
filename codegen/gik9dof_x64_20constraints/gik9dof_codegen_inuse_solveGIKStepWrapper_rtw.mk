###########################################################################
## Makefile generated for component 'gik9dof_codegen_inuse_solveGIKStepWrapper'. 
## 
## Makefile     : gik9dof_codegen_inuse_solveGIKStepWrapper_rtw.mk
## Generated on : Wed Oct 08 13:58:56 2025
## Final product: .\gik9dof_codegen_inuse_solveGIKStepWrapper.lib
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPILER_COMMAND_FILE   Compiler command listing model reference header paths
# CMD_FILE                Command file
# MODELLIB                Static library target

PRODUCT_NAME              = gik9dof_codegen_inuse_solveGIKStepWrapper
MAKEFILE                  = gik9dof_codegen_inuse_solveGIKStepWrapper_rtw.mk
MATLAB_ROOT               = D:\Program Files\MATLAB\R2024b
MATLAB_BIN                = D:\Program Files\MATLAB\R2024b\bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)\win64
START_DIR                 = H:\wSpace\codegenGIKsample\Trial\gikWBC9DOF\codegen\gik9dof_x64_20constraints
TGT_FCN_LIB               = ISO_C++11
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = .
COMPILER_COMMAND_FILE     = gik9dof_codegen_inuse_solveGIKStepWrapper_rtw_comp.rsp
CMD_FILE                  = gik9dof_codegen_inuse_solveGIKStepWrapper_rtw.rsp
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 
NODEBUG                   = 1
MODELLIB                  = gik9dof_codegen_inuse_solveGIKStepWrapper.lib

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          Microsoft Visual C++ 2022 v17.0 | nmake (64-bit Windows)
# Supported Version(s):    17.0
# ToolchainInfo Version:   2024b
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# NODEBUG
# cvarsdll
# cvarsmt
# conlibsmt
# ldebug
# conflags
# cflags

#-----------
# MACROS
#-----------

MW_EXTERNLIB_DIR    = $(MATLAB_ROOT)\extern\lib\win64\microsoft
MW_LIB_DIR          = $(MATLAB_ROOT)\lib\win64
CPU                 = AMD64
APPVER              = 5.02
CVARSFLAG           = $(cvarsmt)
CFLAGS_ADDITIONAL   = -D_CRT_SECURE_NO_WARNINGS
CPPFLAGS_ADDITIONAL = -EHs -D_CRT_SECURE_NO_WARNINGS /wd4251 /Zc:__cplusplus
LIBS_TOOLCHAIN      = $(conlibs)

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = 

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: Microsoft Visual C Compiler
CC = cl

# Linker: Microsoft Visual C Linker
LD = link

# C++ Compiler: Microsoft Visual C++ Compiler
CPP = cl

# C++ Linker: Microsoft Visual C++ Linker
CPP_LD = link

# Archiver: Microsoft Visual C/C++ Archiver
AR = lib

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)\mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: NMAKE Utility
MAKE = nmake


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -Zi
C_OUTPUT_FLAG       = -Fo
LDDEBUG             = /DEBUG
OUTPUT_FLAG         = -out:
CPPDEBUG            = -Zi
CPP_OUTPUT_FLAG     = -Fo
CPPLDDEBUG          = /DEBUG
OUTPUT_FLAG         = -out:
ARDEBUG             =
STATICLIB_OUTPUT_FLAG = -out:
MEX_DEBUG           = -g
RM                  = @del
ECHO                = @echo
MV                  = @ren
RUN                 = @cmd /C

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = /nologo
CFLAGS               = $(cflags) $(CVARSFLAG) $(CFLAGS_ADDITIONAL) \
                       /O2 /Oy-
CPPFLAGS             = /TP $(cflags) $(CVARSFLAG) $(CPPFLAGS_ADDITIONAL) \
                       /O2 /Oy-
CPP_LDFLAGS          = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN)
CPP_SHAREDLIB_LDFLAGS  = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN) \
                         -dll -def:$(DEF_FILE)
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN)
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN) \
                       -dll -def:$(DEF_FILE)



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = .\gik9dof_codegen_inuse_solveGIKStepWrapper.lib
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = 

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -Dccd_EXPORTS
DEFINES_CUSTOM = 
DEFINES_STANDARD = -DMODEL=gik9dof_codegen_inuse_solveGIKStepWrapper

DEFINES = $(DEFINES_) $(DEFINES_CUSTOM) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src\ccd_ccd.c $(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src\ccd_mpr.c $(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src\ccd_polytope.c $(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src\ccd_vec3.c $(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen\collisioncodegen_CollisionGeometry.cpp $(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen\collisioncodegen_api.cpp $(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen\collisioncodegen_ccdExtensions.cpp $(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen\collisioncodegen_checkCollision.cpp $(START_DIR)\coder_posix_time.c $(START_DIR)\gik9dof_codegen_inuse_solveGIKStepWrapper_data.cpp $(START_DIR)\rt_nonfinite.cpp $(START_DIR)\rtGetNaN.cpp $(START_DIR)\rtGetInf.cpp $(START_DIR)\gik9dof_codegen_inuse_solveGIKStepWrapper_initialize.cpp $(START_DIR)\gik9dof_codegen_inuse_solveGIKStepWrapper_terminate.cpp $(START_DIR)\gik9dof_codegen_inuse_solveGIKStepWrapper.cpp $(START_DIR)\buildRobotForCodegen.cpp $(START_DIR)\rand.cpp $(START_DIR)\eml_rand_mt19937ar.cpp $(START_DIR)\xzlangeM.cpp $(START_DIR)\xnrm2.cpp $(START_DIR)\xdotc.cpp $(START_DIR)\validatestring.cpp $(START_DIR)\diff.cpp $(START_DIR)\svd.cpp $(START_DIR)\eye.cpp $(START_DIR)\find.cpp $(START_DIR)\ixfun.cpp $(START_DIR)\tic.cpp $(START_DIR)\norm.cpp $(START_DIR)\toc.cpp $(START_DIR)\randn.cpp $(START_DIR)\ismember.cpp $(START_DIR)\eml_rand_mt19937ar_stateful.cpp $(START_DIR)\strcmp.cpp $(START_DIR)\xaxpy.cpp $(START_DIR)\xrotg.cpp $(START_DIR)\sqrt.cpp $(START_DIR)\xrot.cpp $(START_DIR)\xswap.cpp $(START_DIR)\mldivide.cpp $(START_DIR)\xgeqp3.cpp $(START_DIR)\sort.cpp $(START_DIR)\sortIdx.cpp $(START_DIR)\xzlascl.cpp $(START_DIR)\GIKProblem.cpp $(START_DIR)\ErrorDampedLevenbergMarquardt.cpp $(START_DIR)\rigidBodyJoint.cpp $(START_DIR)\structConstructorHelper.cpp $(START_DIR)\KinematicConstraint.cpp $(START_DIR)\RigidBodyTree.cpp $(START_DIR)\CollisionSet.cpp $(START_DIR)\RigidBody.cpp $(START_DIR)\constraintDistanceBounds.cpp $(START_DIR)\constraintPoseTarget.cpp $(START_DIR)\constraintJointBounds.cpp $(START_DIR)\CoderTimeAPI.cpp $(START_DIR)\rigidBodyTree1.cpp $(START_DIR)\rigidBody1.cpp $(START_DIR)\generalizedInverseKinematics.cpp $(START_DIR)\PoseTarget.cpp $(START_DIR)\JointPositionBounds.cpp $(START_DIR)\DistanceBoundsConstraint.cpp

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = ccd_ccd.obj ccd_mpr.obj ccd_polytope.obj ccd_vec3.obj collisioncodegen_CollisionGeometry.obj collisioncodegen_api.obj collisioncodegen_ccdExtensions.obj collisioncodegen_checkCollision.obj coder_posix_time.obj gik9dof_codegen_inuse_solveGIKStepWrapper_data.obj rt_nonfinite.obj rtGetNaN.obj rtGetInf.obj gik9dof_codegen_inuse_solveGIKStepWrapper_initialize.obj gik9dof_codegen_inuse_solveGIKStepWrapper_terminate.obj gik9dof_codegen_inuse_solveGIKStepWrapper.obj buildRobotForCodegen.obj rand.obj eml_rand_mt19937ar.obj xzlangeM.obj xnrm2.obj xdotc.obj validatestring.obj diff.obj svd.obj eye.obj find.obj ixfun.obj tic.obj norm.obj toc.obj randn.obj ismember.obj eml_rand_mt19937ar_stateful.obj strcmp.obj xaxpy.obj xrotg.obj sqrt.obj xrot.obj xswap.obj mldivide.obj xgeqp3.obj sort.obj sortIdx.obj xzlascl.obj GIKProblem.obj ErrorDampedLevenbergMarquardt.obj rigidBodyJoint.obj structConstructorHelper.obj KinematicConstraint.obj RigidBodyTree.obj CollisionSet.obj RigidBody.obj constraintDistanceBounds.obj constraintPoseTarget.obj constraintJointBounds.obj CoderTimeAPI.obj rigidBodyTree1.obj rigidBody1.obj generalizedInverseKinematics.obj PoseTarget.obj JointPositionBounds.obj DistanceBoundsConstraint.obj

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

SYSTEM_LIBS = /LIBPATH:"$(MATLAB_ROOT)\bin\win64" "$(MATLAB_ROOT)\bin\win64\libiomp5md.lib"

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_ = /source-charset:utf-8
CFLAGS_OPTS = /openmp /wd4101
CFLAGS_BASIC = $(DEFINES) @$(COMPILER_COMMAND_FILE)

CFLAGS = $(CFLAGS) $(CFLAGS_) $(CFLAGS_OPTS) $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_ = /source-charset:utf-8
CPPFLAGS_OPTS = /openmp /wd4101
CPPFLAGS_BASIC = $(DEFINES) @$(COMPILER_COMMAND_FILE)

CPPFLAGS = $(CPPFLAGS) $(CPPFLAGS_) $(CPPFLAGS_OPTS) $(CPPFLAGS_BASIC)

#---------------
# C++ Linker
#---------------

CPP_LDFLAGS_ = /nodefaultlib:vcomp  

CPP_LDFLAGS = $(CPP_LDFLAGS) $(CPP_LDFLAGS_)

#------------------------------
# C++ Shared Library Linker
#------------------------------

CPP_SHAREDLIB_LDFLAGS_ = /nodefaultlib:vcomp  

CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS) $(CPP_SHAREDLIB_LDFLAGS_)

#-----------
# Linker
#-----------

LDFLAGS_ = /nodefaultlib:vcomp  

LDFLAGS = $(LDFLAGS) $(LDFLAGS_)

#--------------------------
# Shared Library Linker
#--------------------------

SHAREDLIB_LDFLAGS_ = /nodefaultlib:vcomp  

SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS) $(SHAREDLIB_LDFLAGS_)

###########################################################################
## INLINED COMMANDS
###########################################################################


!include $(MATLAB_ROOT)\rtw\c\tools\vcdefs.mak


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute set_environment_variables


all : build
	@cmd /C "@echo ### Successfully generated all binary outputs."


build : set_environment_variables prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


set_environment_variables : 
	@set INCLUDE=$(INCLUDES);$(INCLUDE)
	@set LIB=$(LIB)


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	@cmd /C "@echo ### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS) -out:$(PRODUCT) @$(CMD_FILE)
	@cmd /C "@echo ### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(RELATIVE_PATH_TO_ANCHOR)}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(RELATIVE_PATH_TO_ANCHOR)}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(RELATIVE_PATH_TO_ANCHOR)}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(RELATIVE_PATH_TO_ANCHOR)}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(START_DIR)}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(START_DIR)}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(START_DIR)}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(START_DIR)}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{H:\wSpace\codegenGIKsample\Trial\gikWBC9DOF\matlab}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{H:\wSpace\codegenGIKsample\Trial\gikWBC9DOF\matlab}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{H:\wSpace\codegenGIKsample\Trial\gikWBC9DOF\matlab}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{H:\wSpace\codegenGIKsample\Trial\gikWBC9DOF\matlab}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen}.c.obj:
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen}.cpp.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen}.cc.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen}.cxx.obj:
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


ccd_ccd.obj : "$(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src\ccd_ccd.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src\ccd_ccd.c"


ccd_mpr.obj : "$(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src\ccd_mpr.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src\ccd_mpr.c"


ccd_polytope.obj : "$(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src\ccd_polytope.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src\ccd_polytope.c"


ccd_vec3.obj : "$(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src\ccd_vec3.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(MATLAB_ROOT)\toolbox\shared\robotics\externalDependency\libccd\src\ccd_vec3.c"


collisioncodegen_CollisionGeometry.obj : "$(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen\collisioncodegen_CollisionGeometry.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen\collisioncodegen_CollisionGeometry.cpp"


collisioncodegen_api.obj : "$(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen\collisioncodegen_api.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen\collisioncodegen_api.cpp"


collisioncodegen_ccdExtensions.obj : "$(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen\collisioncodegen_ccdExtensions.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen\collisioncodegen_ccdExtensions.cpp"


collisioncodegen_checkCollision.obj : "$(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen\collisioncodegen_checkCollision.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(MATLAB_ROOT)\toolbox\shared\robotics\robotcore\builtins\libsrc\collisioncodegen\collisioncodegen_checkCollision.cpp"


coder_posix_time.obj : "$(START_DIR)\coder_posix_time.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\coder_posix_time.c"


gik9dof_codegen_inuse_solveGIKStepWrapper_data.obj : "$(START_DIR)\gik9dof_codegen_inuse_solveGIKStepWrapper_data.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\gik9dof_codegen_inuse_solveGIKStepWrapper_data.cpp"


rt_nonfinite.obj : "$(START_DIR)\rt_nonfinite.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\rt_nonfinite.cpp"


rtGetNaN.obj : "$(START_DIR)\rtGetNaN.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\rtGetNaN.cpp"


rtGetInf.obj : "$(START_DIR)\rtGetInf.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\rtGetInf.cpp"


gik9dof_codegen_inuse_solveGIKStepWrapper_initialize.obj : "$(START_DIR)\gik9dof_codegen_inuse_solveGIKStepWrapper_initialize.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\gik9dof_codegen_inuse_solveGIKStepWrapper_initialize.cpp"


gik9dof_codegen_inuse_solveGIKStepWrapper_terminate.obj : "$(START_DIR)\gik9dof_codegen_inuse_solveGIKStepWrapper_terminate.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\gik9dof_codegen_inuse_solveGIKStepWrapper_terminate.cpp"


gik9dof_codegen_inuse_solveGIKStepWrapper.obj : "$(START_DIR)\gik9dof_codegen_inuse_solveGIKStepWrapper.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\gik9dof_codegen_inuse_solveGIKStepWrapper.cpp"


buildRobotForCodegen.obj : "$(START_DIR)\buildRobotForCodegen.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\buildRobotForCodegen.cpp"


rand.obj : "$(START_DIR)\rand.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\rand.cpp"


eml_rand_mt19937ar.obj : "$(START_DIR)\eml_rand_mt19937ar.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\eml_rand_mt19937ar.cpp"


xzlangeM.obj : "$(START_DIR)\xzlangeM.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\xzlangeM.cpp"


xnrm2.obj : "$(START_DIR)\xnrm2.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\xnrm2.cpp"


xdotc.obj : "$(START_DIR)\xdotc.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\xdotc.cpp"


validatestring.obj : "$(START_DIR)\validatestring.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\validatestring.cpp"


diff.obj : "$(START_DIR)\diff.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\diff.cpp"


svd.obj : "$(START_DIR)\svd.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\svd.cpp"


eye.obj : "$(START_DIR)\eye.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\eye.cpp"


find.obj : "$(START_DIR)\find.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\find.cpp"


ixfun.obj : "$(START_DIR)\ixfun.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\ixfun.cpp"


tic.obj : "$(START_DIR)\tic.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\tic.cpp"


norm.obj : "$(START_DIR)\norm.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\norm.cpp"


toc.obj : "$(START_DIR)\toc.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\toc.cpp"


randn.obj : "$(START_DIR)\randn.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\randn.cpp"


ismember.obj : "$(START_DIR)\ismember.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\ismember.cpp"


eml_rand_mt19937ar_stateful.obj : "$(START_DIR)\eml_rand_mt19937ar_stateful.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\eml_rand_mt19937ar_stateful.cpp"


strcmp.obj : "$(START_DIR)\strcmp.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\strcmp.cpp"


xaxpy.obj : "$(START_DIR)\xaxpy.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\xaxpy.cpp"


xrotg.obj : "$(START_DIR)\xrotg.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\xrotg.cpp"


sqrt.obj : "$(START_DIR)\sqrt.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\sqrt.cpp"


xrot.obj : "$(START_DIR)\xrot.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\xrot.cpp"


xswap.obj : "$(START_DIR)\xswap.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\xswap.cpp"


mldivide.obj : "$(START_DIR)\mldivide.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\mldivide.cpp"


xgeqp3.obj : "$(START_DIR)\xgeqp3.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\xgeqp3.cpp"


sort.obj : "$(START_DIR)\sort.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\sort.cpp"


sortIdx.obj : "$(START_DIR)\sortIdx.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\sortIdx.cpp"


xzlascl.obj : "$(START_DIR)\xzlascl.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\xzlascl.cpp"


GIKProblem.obj : "$(START_DIR)\GIKProblem.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\GIKProblem.cpp"


ErrorDampedLevenbergMarquardt.obj : "$(START_DIR)\ErrorDampedLevenbergMarquardt.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\ErrorDampedLevenbergMarquardt.cpp"


rigidBodyJoint.obj : "$(START_DIR)\rigidBodyJoint.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\rigidBodyJoint.cpp"


structConstructorHelper.obj : "$(START_DIR)\structConstructorHelper.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\structConstructorHelper.cpp"


KinematicConstraint.obj : "$(START_DIR)\KinematicConstraint.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\KinematicConstraint.cpp"


RigidBodyTree.obj : "$(START_DIR)\RigidBodyTree.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\RigidBodyTree.cpp"


CollisionSet.obj : "$(START_DIR)\CollisionSet.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\CollisionSet.cpp"


RigidBody.obj : "$(START_DIR)\RigidBody.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\RigidBody.cpp"


constraintDistanceBounds.obj : "$(START_DIR)\constraintDistanceBounds.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\constraintDistanceBounds.cpp"


constraintPoseTarget.obj : "$(START_DIR)\constraintPoseTarget.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\constraintPoseTarget.cpp"


constraintJointBounds.obj : "$(START_DIR)\constraintJointBounds.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\constraintJointBounds.cpp"


CoderTimeAPI.obj : "$(START_DIR)\CoderTimeAPI.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\CoderTimeAPI.cpp"


rigidBodyTree1.obj : "$(START_DIR)\rigidBodyTree1.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\rigidBodyTree1.cpp"


rigidBody1.obj : "$(START_DIR)\rigidBody1.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\rigidBody1.cpp"


generalizedInverseKinematics.obj : "$(START_DIR)\generalizedInverseKinematics.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\generalizedInverseKinematics.cpp"


PoseTarget.obj : "$(START_DIR)\PoseTarget.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\PoseTarget.cpp"


JointPositionBounds.obj : "$(START_DIR)\JointPositionBounds.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\JointPositionBounds.cpp"


DistanceBoundsConstraint.obj : "$(START_DIR)\DistanceBoundsConstraint.cpp"
	$(CPP) $(CPPFLAGS) -Fo"$@" "$(START_DIR)\DistanceBoundsConstraint.cpp"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(COMPILER_COMMAND_FILE) $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@cmd /C "@echo ### PRODUCT = $(PRODUCT)"
	@cmd /C "@echo ### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@cmd /C "@echo ### BUILD_TYPE = $(BUILD_TYPE)"
	@cmd /C "@echo ### INCLUDES = $(INCLUDES)"
	@cmd /C "@echo ### DEFINES = $(DEFINES)"
	@cmd /C "@echo ### ALL_SRCS = $(ALL_SRCS)"
	@cmd /C "@echo ### ALL_OBJS = $(ALL_OBJS)"
	@cmd /C "@echo ### LIBS = $(LIBS)"
	@cmd /C "@echo ### MODELREF_LIBS = $(MODELREF_LIBS)"
	@cmd /C "@echo ### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@cmd /C "@echo ### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@cmd /C "@echo ### CFLAGS = $(CFLAGS)"
	@cmd /C "@echo ### LDFLAGS = $(LDFLAGS)"
	@cmd /C "@echo ### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@cmd /C "@echo ### CPPFLAGS = $(CPPFLAGS)"
	@cmd /C "@echo ### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@cmd /C "@echo ### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@cmd /C "@echo ### ARFLAGS = $(ARFLAGS)"
	@cmd /C "@echo ### MEX_CFLAGS = $(MEX_CFLAGS)"
	@cmd /C "@echo ### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@cmd /C "@echo ### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@cmd /C "@echo ### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@cmd /C "@echo ### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@cmd /C "@echo ### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@cmd /C "@echo ### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files ..."
	@if exist $(PRODUCT) $(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(ECHO) "### Deleted all derived files."


