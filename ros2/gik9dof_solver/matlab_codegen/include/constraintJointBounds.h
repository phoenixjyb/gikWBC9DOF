//
// File: constraintJointBounds.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

#ifndef CONSTRAINTJOINTBOUNDS_H
#define CONSTRAINTJOINTBOUNDS_H

// Include Files
#include "gik9dof_codegen_realtime_solveGIKStepWrapper_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
namespace coder {
class rigidBodyTree;

}
} // namespace gik9dof

// Type Definitions
namespace gik9dof {
namespace coder {
class constraintJointBounds {
public:
  constraintJointBounds *init(rigidBodyTree &rigidbodytree);
  constraintJointBounds();
  ~constraintJointBounds();
  ::coder::array<double, 2U> BoundsInternal;
  ::coder::array<double, 2U> WeightsInternal;

protected:
  cell_49 ConstructorPropertyDefaultValues;

private:
  double NumElements;
};

} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for constraintJointBounds.h
//
// [EOF]
//
