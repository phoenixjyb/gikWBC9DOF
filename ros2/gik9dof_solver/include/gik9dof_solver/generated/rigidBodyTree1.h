//
// rigidBodyTree1.h
//
// Code generation for function 'rigidBodyTree1'
//

#ifndef RIGIDBODYTREE1_H
#define RIGIDBODYTREE1_H

// Include files
#include "CollisionSet.h"
#include "RigidBodyTree.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class rigidBody;

class rigidBodyJoint;

namespace robotics {
namespace manip {
namespace internal {
class RigidBody;

}
} // namespace manip
} // namespace robotics
} // namespace coder

// Type Definitions
namespace coder {
class rigidBodyTree {
public:
  rigidBodyTree *init();
  void addBody(rigidBody &bodyin,
               robotics::manip::internal::CollisionSet &iobj_0,
               rigidBodyJoint &iobj_1,
               robotics::manip::internal::RigidBody &iobj_2);
  void b_addBody(rigidBody &bodyin,
                 robotics::manip::internal::CollisionSet &iobj_0,
                 rigidBodyJoint &iobj_1,
                 robotics::manip::internal::RigidBody &iobj_2);
  void c_addBody(rigidBody &bodyin,
                 robotics::manip::internal::CollisionSet &iobj_0,
                 rigidBodyJoint &iobj_1,
                 robotics::manip::internal::RigidBody &iobj_2);
  void d_addBody(rigidBody &bodyin,
                 robotics::manip::internal::CollisionSet &iobj_0,
                 rigidBodyJoint &iobj_1,
                 robotics::manip::internal::RigidBody &iobj_2);
  void e_addBody(rigidBody &bodyin,
                 robotics::manip::internal::CollisionSet &iobj_0,
                 rigidBodyJoint &iobj_1,
                 robotics::manip::internal::RigidBody &iobj_2);
  void f_addBody(rigidBody &bodyin,
                 robotics::manip::internal::CollisionSet &iobj_0,
                 rigidBodyJoint &iobj_1,
                 robotics::manip::internal::RigidBody &iobj_2);
  void g_addBody(rigidBody &bodyin,
                 robotics::manip::internal::CollisionSet &iobj_0,
                 rigidBodyJoint &iobj_1,
                 robotics::manip::internal::RigidBody &iobj_2);
  void h_addBody(rigidBody &bodyin,
                 robotics::manip::internal::CollisionSet &iobj_0,
                 rigidBodyJoint &iobj_1,
                 robotics::manip::internal::RigidBody &iobj_2);
  void i_addBody(rigidBody &bodyin,
                 robotics::manip::internal::CollisionSet &iobj_0,
                 rigidBodyJoint &iobj_1,
                 robotics::manip::internal::RigidBody &iobj_2);
  void j_addBody(rigidBody &bodyin,
                 robotics::manip::internal::CollisionSet &iobj_0,
                 rigidBodyJoint &iobj_1,
                 robotics::manip::internal::RigidBody &iobj_2);
  void k_addBody(rigidBody &bodyin,
                 robotics::manip::internal::CollisionSet &iobj_0,
                 rigidBodyJoint &iobj_1,
                 robotics::manip::internal::RigidBody &iobj_2);
  void get_BaseName(char basename_data[], int basename_size[2]) const;
  rigidBodyTree();
  ~rigidBodyTree();
  bool matlabCodegenIsDeleted;
  robotics::manip::internal::b_RigidBodyTree TreeInternal;
  robotics::manip::internal::CollisionSet _pobj0;
};

} // namespace coder

#endif
// End of code generation (rigidBodyTree1.h)
