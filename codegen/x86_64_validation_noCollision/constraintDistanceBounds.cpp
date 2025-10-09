//
// File: constraintDistanceBounds.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "constraintDistanceBounds.h"
#include "rt_nonfinite.h"
#include "solveGIKStepWrapper_data.h"
#include "solveGIKStepWrapper_types1.h"
#include "coder_array.h"
#include <cstring>

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
  obj->EndEffector.set_size(1, 17);
  for (int i{0}; i < 17; i++) {
    obj->EndEffector[i] = cv[i];
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
