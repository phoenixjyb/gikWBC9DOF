//
// PoseTarget.h
//
// Code generation for function 'PoseTarget'
//

#ifndef POSETARGET_H
#define POSETARGET_H

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
  b_RigidBodyTree *Tree;
  array<double, 2U> BoundsInternal;
  array<double, 2U> Weights;
  double ReferenceBodyIndex;
  double EndEffectorIndex;
  double TargetTransform[16];
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

#endif
// End of code generation (PoseTarget.h)
