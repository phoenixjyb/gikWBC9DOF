//
// File: PoseTarget.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

#ifndef POSETARGET_H
#define POSETARGET_H

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
class PoseTarget {
public:
  PoseTarget *init(RigidBodyTree *tree);
  void evaluate(const array<double, 1U> &q, double g[2], double J_data[],
                int J_size[2]);
  void evaluateFromTransform(const double T_data[], const int T_size[2],
                             double g[2], double JTwist[12]) const;
  void get_EndEffector(char value_data[], int value_size[2]);
  void get_ReferenceBody(char value_data[], int value_size[2]);
  boolean_T matlabCodegenIsDeleted;
  double NumElements;
  RigidBodyTree *Tree;
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
//
// File trailer for PoseTarget.h
//
// [EOF]
//
