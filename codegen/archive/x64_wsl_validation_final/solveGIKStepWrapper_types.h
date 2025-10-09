//
// File: solveGIKStepWrapper_types.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

#ifndef SOLVEGIKSTEPWRAPPER_TYPES_H
#define SOLVEGIKSTEPWRAPPER_TYPES_H

// Include Files
#include "CollisionSet.h"
#include "RigidBody.h"
#include "constraintDistanceBounds.h"
#include "constraintJointBounds.h"
#include "constraintPoseTarget.h"
#include "generalizedInverseKinematics.h"
#include "rigidBodyJoint.h"
#include "rigidBodyTree1.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include "coder_bounded_array.h"
#define MAX_THREADS omp_get_max_threads()

// Type Definitions
namespace gik9dof {
struct struct1_T {
  ::coder::bounded_array<char, 8U, 2U> Type;
  ::coder::array<double, 2U> Violation;
};

struct struct0_T {
  double Iterations;
  double NumRandomRestarts;
  struct1_T ConstraintViolations[22];
  double ExitFlag;
  ::coder::bounded_array<char, 14U, 2U> Status;
};

struct solveGIKStepWrapperPersistentData {
  coder::robotics::manip::internal::RigidBody gobj_7[11];
  coder::rigidBodyJoint gobj_6[22];
  coder::robotics::manip::internal::CollisionSet gobj_5[22];
  coder::constraintDistanceBounds gobj_4[20];
  coder::generalizedInverseKinematics solver;
  boolean_T solver_not_empty;
  coder::rigidBodyTree robot;
  coder::constraintPoseTarget poseConstraint;
  coder::constraintJointBounds jointConstraint;
  coder::constraintDistanceBounds *distConstraints[20];
  unsigned int state[625];
  double freq;
  boolean_T freq_not_empty;
  double lims[2];
};

struct solveGIKStepWrapperStackData {
  solveGIKStepWrapperPersistentData *pd;
};

} // namespace gik9dof

#endif
//
// File trailer for solveGIKStepWrapper_types.h
//
// [EOF]
//
