//
// File: buildRobotForCodegen.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 19:17:46
//

#ifndef BUILDROBOTFORCODEGEN_H
#define BUILDROBOTFORCODEGEN_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
class GIKSolver;

namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class CollisionSet;

}
} // namespace manip
} // namespace robotics
class rigidBodyJoint;

namespace robotics {
namespace manip {
namespace internal {
class RigidBody;

}
} // namespace manip
} // namespace robotics
class rigidBodyTree;

} // namespace coder
} // namespace gik9dof

// Function Declarations
namespace gik9dof {
namespace codegen_inuse {
coder::rigidBodyTree *
buildRobotForCodegen(GIKSolver *aInstancePtr,
                     coder::robotics::manip::internal::CollisionSet &iobj_0,
                     coder::rigidBodyJoint &iobj_1,
                     coder::robotics::manip::internal::RigidBody &iobj_2,
                     coder::rigidBodyTree &iobj_3);

}
} // namespace gik9dof

#endif
//
// File trailer for buildRobotForCodegen.h
//
// [EOF]
//
