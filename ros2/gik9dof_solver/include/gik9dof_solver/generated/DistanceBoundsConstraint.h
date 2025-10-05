//
// DistanceBoundsConstraint.h
//
// Code generation for function 'DistanceBoundsConstraint'
//

#ifndef DISTANCEBOUNDSCONSTRAINT_H
#define DISTANCEBOUNDSCONSTRAINT_H

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
class DistanceBoundsConstraint {
public:
  void get_EndEffector(char value_data[], int value_size[2]);
  void get_ReferenceBody(char value_data[], int value_size[2]);
  DistanceBoundsConstraint();
  ~DistanceBoundsConstraint();
  bool matlabCodegenIsDeleted;
  double NumElements;
  b_RigidBodyTree *Tree;
  array<double, 2U> BoundsInternal;
  array<double, 2U> Weights;
  double ReferenceBodyIndex;
  double EndEffectorIndex;
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

#endif
// End of code generation (DistanceBoundsConstraint.h)
