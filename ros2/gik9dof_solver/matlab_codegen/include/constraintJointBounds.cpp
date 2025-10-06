//
// File: constraintJointBounds.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

// Include Files
#include "constraintJointBounds.h"
#include "RigidBodyTree.h"
#include "gik9dof_codegen_realtime_solveGIKStepWrapper_types.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : void
// Return Type  : constraintJointBounds
//
namespace gik9dof {
namespace coder {
constraintJointBounds::constraintJointBounds() = default;

//
// Arguments    : void
// Return Type  : void
//
constraintJointBounds::~constraintJointBounds() = default;

//
// Arguments    : rigidBodyTree &rigidbodytree
// Return Type  : constraintJointBounds *
//
constraintJointBounds *constraintJointBounds::init(rigidBodyTree &rigidbodytree)
{
  constraintJointBounds *obj;
  ::coder::array<double, 2U> defaultValues_f2;
  ::coder::array<double, 2U> limits;
  int i;
  int loop_ub;
  int loop_ub_tmp;
  obj = this;
  rigidbodytree.TreeInternal.get_JointPositionLimits(limits);
  obj->BoundsInternal.set_size(limits.size(0), 2);
  loop_ub_tmp = limits.size(0) << 1;
  for (i = 0; i < loop_ub_tmp; i++) {
    obj->BoundsInternal[i] = limits[i];
  }
  obj->NumElements = obj->BoundsInternal.size(0);
  loop_ub_tmp = static_cast<int>(obj->NumElements);
  obj->WeightsInternal.set_size(1, loop_ub_tmp);
  for (i = 0; i < loop_ub_tmp; i++) {
    obj->WeightsInternal[i] = 1.0;
  }
  i = obj->BoundsInternal.size(0);
  limits.set_size(i, 2);
  loop_ub = obj->BoundsInternal.size(0) << 1;
  for (int i1{0}; i1 < loop_ub; i1++) {
    limits[i1] = obj->BoundsInternal[i1];
  }
  loop_ub = obj->WeightsInternal.size(1);
  defaultValues_f2.set_size(1, loop_ub);
  loop_ub_tmp = obj->WeightsInternal.size(1);
  for (int i1{0}; i1 < loop_ub_tmp; i1++) {
    defaultValues_f2[i1] = obj->WeightsInternal[i1];
  }
  obj->ConstructorPropertyDefaultValues.f1.set_size(i, 2);
  loop_ub_tmp = limits.size(0) << 1;
  for (i = 0; i < loop_ub_tmp; i++) {
    obj->ConstructorPropertyDefaultValues.f1[i] = limits[i];
  }
  obj->ConstructorPropertyDefaultValues.f2.set_size(1, loop_ub);
  for (i = 0; i < loop_ub; i++) {
    obj->ConstructorPropertyDefaultValues.f2[i] = defaultValues_f2[i];
  }
  i = obj->ConstructorPropertyDefaultValues.f1.size(0);
  limits.set_size(i, 2);
  loop_ub = obj->ConstructorPropertyDefaultValues.f1.size(0) << 1;
  for (int i1{0}; i1 < loop_ub; i1++) {
    limits[i1] = obj->ConstructorPropertyDefaultValues.f1[i1];
  }
  loop_ub = obj->ConstructorPropertyDefaultValues.f2.size(1);
  defaultValues_f2.set_size(1, loop_ub);
  loop_ub_tmp = obj->ConstructorPropertyDefaultValues.f2.size(1);
  for (int i1{0}; i1 < loop_ub_tmp; i1++) {
    defaultValues_f2[i1] = obj->ConstructorPropertyDefaultValues.f2[i1];
  }
  obj->BoundsInternal.set_size(i, 2);
  loop_ub_tmp = limits.size(0) << 1;
  for (i = 0; i < loop_ub_tmp; i++) {
    obj->BoundsInternal[i] = limits[i];
  }
  obj->WeightsInternal.set_size(1, loop_ub);
  for (i = 0; i < loop_ub; i++) {
    obj->WeightsInternal[i] = defaultValues_f2[i];
  }
  return obj;
}

} // namespace coder
} // namespace gik9dof

//
// File trailer for constraintJointBounds.cpp
//
// [EOF]
//
