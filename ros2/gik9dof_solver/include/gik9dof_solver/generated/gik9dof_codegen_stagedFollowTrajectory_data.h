//
// gik9dof_codegen_stagedFollowTrajectory_data.h
//
// Code generation for function 'gik9dof_codegen_stagedFollowTrajectory_data'
//

#ifndef GIK9DOF_CODEGEN_STAGEDFOLLOWTRAJECTORY_DATA_H
#define GIK9DOF_CODEGEN_STAGEDFOLLOWTRAJECTORY_DATA_H

// Include files
#include "constraintDistanceBounds.h"
#include "constraintJointBounds.h"
#include "constraintPoseTarget.h"
#include "generalizedInverseKinematics.h"
#include "gik9dof_codegen_stagedFollowTrajectory_types.h"
#include "rigidBodyTree1.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include "omp.h"
#include <cstddef>
#include <cstdlib>

// Variable Declarations
extern coder::rigidBodyTree robot;
extern unsigned int state[625];
extern coder::generalizedInverseKinematics solver;
extern bool solver_not_empty;
extern coder::constraintPoseTarget poseConstraint;
extern coder::constraintJointBounds jointConstraint;
extern coder::constraintDistanceBounds distanceConstraint;
extern coder::rigidBodyTree *b_robot;
extern coder::array<double, 2U> jointBoundsDefault;
extern double freq;
extern bool freq_not_empty;
extern omp_nest_lock_t gik9dof_codegen_stagedFollowTrajectory_nestLockGlobal;
extern const signed char iv[16];
extern const char cv1[9];
extern const char cv2[11];
extern const char cv3[11];
extern const char cv4[21];
extern const char cv5[18];
extern const char cv6[14];
extern const char cv7[14];
extern const char cv8[14];
extern const char cv9[14];
extern const char cv10[14];
extern const char cv11[14];
extern const char cv12[17];
extern bool isInitialized_gik9dof_codegen_stagedFollowTrajectory;

#endif
// End of code generation (gik9dof_codegen_stagedFollowTrajectory_data.h)
