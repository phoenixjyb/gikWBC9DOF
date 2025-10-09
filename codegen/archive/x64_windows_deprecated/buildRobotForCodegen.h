//
// File: buildRobotForCodegen.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
//

#ifndef BUILDROBOTFORCODEGEN_H
#define BUILDROBOTFORCODEGEN_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
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

// Function Declarations
namespace gik9dof {
namespace codegen_inuse {
coder::rigidBodyTree *
buildRobotForCodegen(coder::robotics::manip::internal::CollisionSet &iobj_0,
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
