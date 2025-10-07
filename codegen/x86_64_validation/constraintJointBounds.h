//
// File: constraintJointBounds.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

#ifndef CONSTRAINTJOINTBOUNDS_H
#define CONSTRAINTJOINTBOUNDS_H

// Include Files
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
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
  cell_53 ConstructorPropertyDefaultValues;

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
