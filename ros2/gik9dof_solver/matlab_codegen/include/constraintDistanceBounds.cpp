//
// File: constraintDistanceBounds.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

// Include Files
#include "constraintDistanceBounds.h"
#include "gik9dof_codegen_realtime_solveGIKStepWrapper_data.h"
#include "gik9dof_codegen_realtime_solveGIKStepWrapper_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : void
// Return Type  : constraintDistanceBounds
//
namespace gik9dof {
namespace coder {
constraintDistanceBounds::constraintDistanceBounds() = default;

//
// Arguments    : void
// Return Type  : void
//
constraintDistanceBounds::~constraintDistanceBounds() = default;

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
} // namespace gik9dof

//
// File trailer for constraintDistanceBounds.cpp
//
// [EOF]
//
