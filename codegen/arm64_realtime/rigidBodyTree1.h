//
// File: rigidBodyTree1.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 13:57:39
//

#ifndef RIGIDBODYTREE1_H
#define RIGIDBODYTREE1_H

// Include Files
#include "CollisionSet.h"
#include "RigidBodyTree.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
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
} // namespace gik9dof

// Type Definitions
namespace gik9dof {
namespace coder {
class rigidBodyTree {
public:
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
  rigidBodyTree();
  ~rigidBodyTree();
  bool matlabCodegenIsDeleted;
  robotics::manip::internal::b_RigidBodyTree TreeInternal;
  robotics::manip::internal::CollisionSet _pobj0;
};

} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for rigidBodyTree1.h
//
// [EOF]
//
