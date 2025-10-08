//
// File: JointPositionBounds.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
//

#ifndef JOINTPOSITIONBOUNDS_H
#define JOINTPOSITIONBOUNDS_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class RigidBodyTree;

}
} // namespace manip
} // namespace robotics
} // namespace coder

// Type Definitions
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class JointPositionBounds {
public:
  JointPositionBounds *init(RigidBodyTree *tree);
  void get_KinematicPath(array<double, 2U> &b_value) const;
  boolean_T matlabCodegenIsDeleted;
  double NumElements;
  RigidBodyTree *Tree;
  array<double, 2U> BoundsInternal;
  array<double, 2U> Weights;
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

#endif
//
// File trailer for JointPositionBounds.h
//
// [EOF]
//
