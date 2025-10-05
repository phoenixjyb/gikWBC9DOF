//
// JointPositionBounds.h
//
// Code generation for function 'JointPositionBounds'
//

#ifndef JOINTPOSITIONBOUNDS_H
#define JOINTPOSITIONBOUNDS_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class b_RigidBodyTree;

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
  void get_KinematicPath(array<double, 2U> &b_value) const;
  JointPositionBounds();
  ~JointPositionBounds();
  bool matlabCodegenIsDeleted;
  double NumElements;
  b_RigidBodyTree *Tree;
  array<double, 2U> BoundsInternal;
  array<double, 2U> Weights;
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

#endif
// End of code generation (JointPositionBounds.h)
