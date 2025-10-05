//
// rigidBody1.h
//
// Code generation for function 'rigidBody1'
//

#ifndef RIGIDBODY1_H
#define RIGIDBODY1_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class RigidBody;

class RigidBodyTree;

class CollisionSet;

} // namespace internal
} // namespace manip
} // namespace robotics
class rigidBodyJoint;

} // namespace coder

// Type Definitions
namespace coder {
class rigidBody {
public:
  rigidBody *init(robotics::manip::internal::RigidBodyTree &iobj_0,
                  robotics::manip::internal::CollisionSet &iobj_1,
                  rigidBodyJoint &iobj_2,
                  robotics::manip::internal::RigidBody &iobj_3);
  rigidBody *b_init(robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  rigidBody *c_init(robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  rigidBody *d_init(robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  rigidBody *e_init(robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  rigidBody *f_init(robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  rigidBody *g_init(robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  rigidBody *h_init(robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  rigidBody *i_init(robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  rigidBody *j_init(robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  rigidBody *k_init(robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  void matlabCodegenDestructor();
  ~rigidBody();
  rigidBody();
  bool matlabCodegenIsDeleted;
  robotics::manip::internal::RigidBody *BodyInternal;
  robotics::manip::internal::RigidBodyTree *TreeInternal;
};

} // namespace coder

#endif
// End of code generation (rigidBody1.h)
