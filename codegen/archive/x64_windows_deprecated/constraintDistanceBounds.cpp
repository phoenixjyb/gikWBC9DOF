//
// File: constraintDistanceBounds.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
//

// Include Files
#include "constraintDistanceBounds.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_data.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
//
// Arguments    : void
// Return Type  : constraintDistanceBounds *
//
namespace coder {
constraintDistanceBounds *constraintDistanceBounds::b_init()
{
  constraintDistanceBounds *obj;
  obj = this;
  obj->ConstructorPropertyDefaultValues.f2[0] = 0.0;
  obj->ConstructorPropertyDefaultValues.f2[1] = 0.0;
  obj->ConstructorPropertyDefaultValues.f3 = 1.0;
  obj->EndEffector.set_size(1, 14);
  for (int i{0}; i < 14; i++) {
    obj->EndEffector[i] = cv10[i];
  }
  double d;
  double defaultValues_f3;
  defaultValues_f3 = obj->ConstructorPropertyDefaultValues.f3;
  obj->ReferenceBody.set_size(0, 0);
  d = obj->ConstructorPropertyDefaultValues.f2[0];
  obj->Bounds[0] = d;
  d = obj->ConstructorPropertyDefaultValues.f2[1];
  obj->Bounds[1] = d;
  obj->Weights = defaultValues_f3;
  return obj;
}

//
// Arguments    : void
// Return Type  : constraintDistanceBounds *
//
constraintDistanceBounds *constraintDistanceBounds::c_init()
{
  constraintDistanceBounds *obj;
  obj = this;
  obj->ConstructorPropertyDefaultValues.f2[0] = 0.0;
  obj->ConstructorPropertyDefaultValues.f2[1] = 0.0;
  obj->ConstructorPropertyDefaultValues.f3 = 1.0;
  obj->EndEffector.set_size(1, 14);
  for (int i{0}; i < 14; i++) {
    obj->EndEffector[i] = cv9[i];
  }
  double d;
  double defaultValues_f3;
  defaultValues_f3 = obj->ConstructorPropertyDefaultValues.f3;
  obj->ReferenceBody.set_size(0, 0);
  d = obj->ConstructorPropertyDefaultValues.f2[0];
  obj->Bounds[0] = d;
  d = obj->ConstructorPropertyDefaultValues.f2[1];
  obj->Bounds[1] = d;
  obj->Weights = defaultValues_f3;
  return obj;
}

//
// Arguments    : void
// Return Type  : constraintDistanceBounds *
//
constraintDistanceBounds *constraintDistanceBounds::init()
{
  constraintDistanceBounds *obj;
  obj = this;
  obj->ConstructorPropertyDefaultValues.f2[0] = 0.0;
  obj->ConstructorPropertyDefaultValues.f2[1] = 0.0;
  obj->ConstructorPropertyDefaultValues.f3 = 1.0;
  obj->EndEffector.set_size(1, 17);
  for (int i{0}; i < 17; i++) {
    obj->EndEffector[i] = cv12[i];
  }
  double d;
  double defaultValues_f3;
  defaultValues_f3 = obj->ConstructorPropertyDefaultValues.f3;
  obj->ReferenceBody.set_size(0, 0);
  d = obj->ConstructorPropertyDefaultValues.f2[0];
  obj->Bounds[0] = d;
  d = obj->ConstructorPropertyDefaultValues.f2[1];
  obj->Bounds[1] = d;
  obj->Weights = defaultValues_f3;
  return obj;
}

} // namespace coder

//
// File trailer for constraintDistanceBounds.cpp
//
// [EOF]
//
