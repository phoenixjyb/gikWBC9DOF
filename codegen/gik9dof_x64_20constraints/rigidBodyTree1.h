//
// File: rigidBodyTree1.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
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
  void get_BaseName(char basename_data[], int basename_size[2]) const;
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
  boolean_T matlabCodegenIsDeleted;
  robotics::manip::internal::RigidBodyTree TreeInternal;
  robotics::manip::internal::CollisionSet _pobj0;
};

} // namespace coder

#endif
//
// File trailer for rigidBodyTree1.h
//
// [EOF]
//
