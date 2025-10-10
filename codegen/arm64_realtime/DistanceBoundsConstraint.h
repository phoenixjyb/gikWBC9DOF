//
// File: DistanceBoundsConstraint.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 19:17:46
//

#ifndef DISTANCEBOUNDSCONSTRAINT_H
#define DISTANCEBOUNDSCONSTRAINT_H

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
class constraintDistanceBounds;

} // namespace coder
} // namespace gik9dof

// Type Definitions
namespace gik9dof {
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class DistanceBoundsConstraint {
public:
  DistanceBoundsConstraint *init(b_RigidBodyTree *tree);
  double evaluate(const ::coder::array<double, 1U> &q, double J_data[],
                  int J_size[2]);
  void update(const constraintDistanceBounds *other);
  void get_EndEffector(char value_data[], int value_size[2]);
  void get_ReferenceBody(char value_data[], int value_size[2]);
  DistanceBoundsConstraint();
  ~DistanceBoundsConstraint();
  bool matlabCodegenIsDeleted;
  double NumElements;
  b_RigidBodyTree *Tree;
  ::coder::array<double, 2U> BoundsInternal;
  ::coder::array<double, 2U> Weights;
  double ReferenceBodyIndex;
  double EndEffectorIndex;
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for DistanceBoundsConstraint.h
//
// [EOF]
//
