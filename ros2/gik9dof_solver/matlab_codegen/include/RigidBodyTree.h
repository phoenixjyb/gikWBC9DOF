//
// File: RigidBodyTree.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 09-Oct-2025 12:02:50
//

#ifndef RIGIDBODYTREE_H
#define RIGIDBODYTREE_H

// Include Files
#include "CollisionSet.h"
#include "FastVisualizationHelper.h"
#include "RigidBody.h"
#include "rigidBodyJoint.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
class GIKSolver;

}

// Type Definitions
namespace gik9dof {
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class RigidBodyTree {
public:
  RigidBodyTree *init(GIKSolver *aInstancePtr);
  void matlabCodegenDestructor();
  ~RigidBodyTree();
  RigidBodyTree();
  bool matlabCodegenIsDeleted;
  b_RigidBody Base;
  FastVisualizationHelper b_FastVisualizationHelper;
  b_RigidBody *Bodies[1];
  b_RigidBody _pobj0;
};

class b_RigidBodyTree {
public:
  b_RigidBodyTree *init(GIKSolver *aInstancePtr);
  void addBody(RigidBody *bodyin, const char parentName_data[],
               const int parentName_size[2], CollisionSet &iobj_0,
               rigidBodyJoint &iobj_1, RigidBody &iobj_2);
  double findBodyIndexByName(const ::coder::array<char, 2U> &bodyname);
  double findBodyIndexByJointName(const char jointname_data[],
                                  const int jointname_size[2]);
  void get_JointPositionLimits(::coder::array<double, 2U> &limits);
  void efficientFKAndJacobianForIK(const ::coder::array<double, 1U> &qv,
                                   double bid1, double bid2, double T_data[],
                                   int T_size[2],
                                   ::coder::array<double, 2U> &Jac);
  void kinematicPathInternal(RigidBody *body1, RigidBody *body2,
                             ::coder::array<double, 2U> &indices);
  void ancestorIndices(RigidBody *body, ::coder::array<double, 2U> &indices);
  void kinematicPath(const char body1Name_data[], const int body1Name_size[2],
                     const char body2Name_data[], const int body2Name_size[2],
                     ::coder::array<double, 2U> &indices);
  b_RigidBodyTree();
  ~b_RigidBodyTree();
  bool matlabCodegenIsDeleted;
  double NumBodies;
  RigidBody Base;
  FastVisualizationHelper b_FastVisualizationHelper;
  double Gravity[3];
  RigidBody *Bodies[11];
  double NumNonFixedBodies;
  double PositionNumber;
  double VelocityNumber;
  double PositionDoFMap[22];
  double VelocityDoFMap[22];
  CollisionSet _pobj0[2];
  rigidBodyJoint _pobj1[2];
  RigidBody _pobj2[11];
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

// Function Declarations
namespace gik9dof {
void binary_expand_op_29(bool in1[9], const double in2[9],
                         const ::coder::array<double, 2U> &in3);

void binary_expand_op_30(bool in1[9], const double in2[9],
                         const ::coder::array<double, 2U> &in3);

} // namespace gik9dof

#endif
//
// File trailer for RigidBodyTree.h
//
// [EOF]
//
