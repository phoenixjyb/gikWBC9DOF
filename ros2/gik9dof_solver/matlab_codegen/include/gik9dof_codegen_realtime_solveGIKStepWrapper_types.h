//
// File: gik9dof_codegen_realtime_solveGIKStepWrapper_types.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

#ifndef GIK9DOF_CODEGEN_REALTIME_SOLVEGIKSTEPWRAPPER_TYPES_H
#define GIK9DOF_CODEGEN_REALTIME_SOLVEGIKSTEPWRAPPER_TYPES_H

// Include Files
#include "CollisionSet.h"
#include "RigidBody.h"
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
  struct1_T ConstraintViolations[3];
  double ExitFlag;
  ::coder::bounded_array<char, 14U, 2U> Status;
};

struct cell_49 {
  ::coder::array<double, 2U> f1;
  ::coder::array<double, 2U> f2;
};

struct cell_47 {
  double f2[16];
  double f3;
  double f4;
  double f5[2];
};

struct cell_51 {
  double f2[2];
  double f3;
};

struct gik9dof_codegen_realtime_solveGIKStepWrapperPersistentData {
  coder::robotics::manip::internal::RigidBody gobj_4[11];
  coder::rigidBodyJoint gobj_3[22];
  coder::robotics::manip::internal::CollisionSet gobj_2[22];
  coder::rigidBodyTree robot;
  coder::generalizedInverseKinematics solver;
  boolean_T initialized_not_empty;
  unsigned int state[625];
  double freq;
  boolean_T freq_not_empty;
  double lims[2];
};

struct gik9dof_codegen_realtime_solveGIKStepWrapperStackData {
  gik9dof_codegen_realtime_solveGIKStepWrapperPersistentData *pd;
};

} // namespace gik9dof

#endif
//
// File trailer for gik9dof_codegen_realtime_solveGIKStepWrapper_types.h
//
// [EOF]
//
