//
// File: JointPositionBounds.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 19:17:46
//

#ifndef JOINTPOSITIONBOUNDS_H
#define JOINTPOSITIONBOUNDS_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class b_RigidBodyTree;

}
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

// Type Definitions
namespace gik9dof {
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class JointPositionBounds {
public:
  JointPositionBounds *init(b_RigidBodyTree *tree);
  void get_KinematicPath(::coder::array<double, 2U> &b_value) const;
  JointPositionBounds();
  ~JointPositionBounds();
  bool matlabCodegenIsDeleted;
  double NumElements;
  b_RigidBodyTree *Tree;
  ::coder::array<double, 2U> BoundsInternal;
  ::coder::array<double, 2U> Weights;
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for JointPositionBounds.h
//
// [EOF]
//
