//
// File: constraintJointBounds.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
//

#ifndef CONSTRAINTJOINTBOUNDS_H
#define CONSTRAINTJOINTBOUNDS_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class rigidBodyTree;

}

// Type Definitions
struct cell_12 {
  coder::array<double, 2U> f1;
  coder::array<double, 2U> f2;
};

namespace coder {
class constraintJointBounds {
public:
  constraintJointBounds *init(rigidBodyTree &rigidbodytree);
  array<double, 2U> BoundsInternal;
  array<double, 2U> WeightsInternal;

protected:
  cell_12 ConstructorPropertyDefaultValues;

private:
  double NumElements;
};

} // namespace coder

#endif
//
// File trailer for constraintJointBounds.h
//
// [EOF]
//
