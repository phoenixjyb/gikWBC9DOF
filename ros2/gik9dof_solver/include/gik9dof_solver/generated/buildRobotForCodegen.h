//
// buildRobotForCodegen.h
//
// Code generation for function 'buildRobotForCodegen'
//

#ifndef BUILDROBOTFORCODEGEN_H
#define BUILDROBOTFORCODEGEN_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class rigidBodyTree;

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
} // namespace coder

// Function Declarations
namespace gik9dof {
namespace codegen {
void addBodyWithJoint(coder::rigidBodyTree *tree,
                      coder::robotics::manip::internal::CollisionSet &iobj_0,
                      coder::rigidBodyJoint &iobj_1,
                      coder::robotics::manip::internal::RigidBody &iobj_2);

void b_addBodyWithJoint(coder::rigidBodyTree *tree,
                        coder::robotics::manip::internal::CollisionSet &iobj_0,
                        coder::rigidBodyJoint &iobj_1,
                        coder::robotics::manip::internal::RigidBody &iobj_2);

void c_addBodyWithJoint(coder::rigidBodyTree *tree,
                        coder::robotics::manip::internal::CollisionSet &iobj_0,
                        coder::rigidBodyJoint &iobj_1,
                        coder::robotics::manip::internal::RigidBody &iobj_2);

void d_addBodyWithJoint(coder::rigidBodyTree *tree,
                        coder::robotics::manip::internal::CollisionSet &iobj_0,
                        coder::rigidBodyJoint &iobj_1,
                        coder::robotics::manip::internal::RigidBody &iobj_2);

} // namespace codegen
} // namespace gik9dof

#endif
// End of code generation (buildRobotForCodegen.h)
