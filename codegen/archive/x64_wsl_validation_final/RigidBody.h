//
// File: RigidBody.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

#ifndef RIGIDBODY_H
#define RIGIDBODY_H

// Include Files
#include "CharacterVector.h"
#include "CollisionSet.h"
#include "rigidBodyJoint.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace gik9dof {
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class RigidBody {
public:
  RigidBody *init(const char bodyInput[11], CollisionSet &iobj_0,
                  rigidBodyJoint &iobj_1);
  RigidBody *copy(CollisionSet &iobj_0, rigidBodyJoint &iobj_1,
                  RigidBody &iobj_2);
  RigidBody *init(const char bodyInput[10]);
  void matlabCodegenDestructor();
  ~RigidBody();
  RigidBody();
  boolean_T matlabCodegenIsDeleted;
  CharacterVector NameInternal;
  double Index;
  rigidBodyJoint *JointInternal;
  double ParentIndex;
  double MassInternal;
  double CenterOfMassInternal[3];
  double InertiaInternal[9];
  double SpatialInertia[36];
  CollisionSet *CollisionsInternal;
  CollisionSet _pobj0;
  rigidBodyJoint _pobj1;
};

class b_RigidBody {
public:
  void matlabCodegenDestructor();
  ~b_RigidBody();
  b_RigidBody();
  boolean_T matlabCodegenIsDeleted;
  CharacterVector NameInternal;
  rigidBodyJoint JointInternal;
  CollisionSet CollisionsInternal;
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for RigidBody.h
//
// [EOF]
//
