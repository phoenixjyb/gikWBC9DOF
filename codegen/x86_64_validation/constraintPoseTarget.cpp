//
// File: constraintPoseTarget.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

// Include Files
#include "constraintPoseTarget.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_data.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// Arguments    : void
// Return Type  : constraintPoseTarget
//
namespace gik9dof {
namespace coder {
constraintPoseTarget::constraintPoseTarget() = default;

//
// Arguments    : void
// Return Type  : void
//
constraintPoseTarget::~constraintPoseTarget() = default;

//
// Arguments    : void
// Return Type  : constraintPoseTarget *
//
constraintPoseTarget *constraintPoseTarget::init()
{
  constraintPoseTarget *obj;
  double defaultValues_f3;
  double defaultValues_f4;
  double defaultValues_f5_idx_0;
  double defaultValues_f5_idx_1;
  obj = this;
  for (int i{0}; i < 16; i++) {
    obj->ConstructorPropertyDefaultValues.f2[i] = iv[i];
  }
  obj->ConstructorPropertyDefaultValues.f3 = 0.0;
  obj->ConstructorPropertyDefaultValues.f4 = 0.0;
  obj->ConstructorPropertyDefaultValues.f5[0] = 1.0;
  obj->ConstructorPropertyDefaultValues.f5[1] = 1.0;
  for (int i{0}; i < 17; i++) {
    obj->EndEffector[i] = cv12[i];
  }
  defaultValues_f3 = obj->ConstructorPropertyDefaultValues.f3;
  defaultValues_f4 = obj->ConstructorPropertyDefaultValues.f4;
  defaultValues_f5_idx_0 = obj->ConstructorPropertyDefaultValues.f5[0];
  defaultValues_f5_idx_1 = obj->ConstructorPropertyDefaultValues.f5[1];
  for (int i{0}; i < 16; i++) {
    double d;
    d = obj->ConstructorPropertyDefaultValues.f2[i];
    obj->TargetTransform[i] = d;
  }
  obj->OrientationTolerance = defaultValues_f3;
  obj->PositionTolerance = defaultValues_f4;
  obj->Weights[0] = defaultValues_f5_idx_0;
  obj->Weights[1] = defaultValues_f5_idx_1;
  return obj;
}

} // namespace coder
} // namespace gik9dof

//
// File trailer for constraintPoseTarget.cpp
//
// [EOF]
//
