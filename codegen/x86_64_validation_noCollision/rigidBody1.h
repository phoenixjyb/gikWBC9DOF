//
// File: rigidBody1.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

#ifndef RIGIDBODY1_H
#define RIGIDBODY1_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class RigidBody;

class RigidBodyTree;

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
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

} // namespace coder
} // namespace gik9dof

// Type Definitions
namespace gik9dof {
namespace coder {
class rigidBody {
public:
  rigidBody *init(GIKSolver *aInstancePtr,
                  robotics::manip::internal::RigidBodyTree &iobj_0,
                  robotics::manip::internal::CollisionSet &iobj_1,
                  rigidBodyJoint &iobj_2,
                  robotics::manip::internal::RigidBody &iobj_3);
  void set_Mass();
  void set_CenterOfMass();
  void set_Inertia();
  rigidBody *b_init(GIKSolver *aInstancePtr,
                    robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  rigidBody *c_init(GIKSolver *aInstancePtr,
                    robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  void b_set_CenterOfMass();
  void b_set_Inertia();
  rigidBody *d_init(GIKSolver *aInstancePtr,
                    robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  void b_set_Mass();
  void c_set_CenterOfMass();
  void c_set_Inertia();
  rigidBody *e_init(GIKSolver *aInstancePtr,
                    robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  void c_set_Mass();
  void d_set_CenterOfMass();
  void d_set_Inertia();
  rigidBody *f_init(GIKSolver *aInstancePtr,
                    robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  void d_set_Mass();
  void e_set_CenterOfMass();
  void e_set_Inertia();
  rigidBody *g_init(GIKSolver *aInstancePtr,
                    robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  void e_set_Mass();
  void f_set_CenterOfMass();
  void f_set_Inertia();
  rigidBody *h_init(GIKSolver *aInstancePtr,
                    robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  void f_set_Mass();
  void g_set_CenterOfMass();
  void g_set_Inertia();
  rigidBody *i_init(GIKSolver *aInstancePtr,
                    robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  void g_set_Mass();
  void h_set_CenterOfMass();
  void h_set_Inertia();
  rigidBody *j_init(GIKSolver *aInstancePtr,
                    robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  void h_set_Mass();
  void i_set_CenterOfMass();
  void i_set_Inertia();
  rigidBody *k_init(GIKSolver *aInstancePtr,
                    robotics::manip::internal::RigidBodyTree &iobj_0,
                    robotics::manip::internal::CollisionSet &iobj_1,
                    rigidBodyJoint &iobj_2,
                    robotics::manip::internal::RigidBody &iobj_3);
  void i_set_Mass();
  void j_set_CenterOfMass();
  void j_set_Inertia();
  void matlabCodegenDestructor();
  ~rigidBody();
  rigidBody();
  boolean_T matlabCodegenIsDeleted;
  robotics::manip::internal::RigidBody *BodyInternal;
  robotics::manip::internal::RigidBodyTree *TreeInternal;
};

} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for rigidBody1.h
//
// [EOF]
//
