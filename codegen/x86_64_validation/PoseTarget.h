//
// File: PoseTarget.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

#ifndef POSETARGET_H
#define POSETARGET_H

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
class RigidBodyTree;

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
class PoseTarget {
public:
  void evaluateFromTransform(const double T_data[], const int T_size[2],
                             double g[2], double JTwist[12]) const;
  void get_EndEffector(char value_data[], int value_size[2]);
  void get_ReferenceBody(char value_data[], int value_size[2]);
  PoseTarget();
  ~PoseTarget();
  bool matlabCodegenIsDeleted;
  double NumElements;
  RigidBodyTree *Tree;
  ::coder::array<double, 2U> BoundsInternal;
  ::coder::array<double, 2U> Weights;
  double ReferenceBodyIndex;
  double EndEffectorIndex;
  double TargetTransform[16];
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for PoseTarget.h
//
// [EOF]
//
