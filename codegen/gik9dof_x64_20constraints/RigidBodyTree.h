//
// File: RigidBodyTree.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
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

// Type Definitions
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class RigidBodyTree {
public:
  RigidBodyTree *init();
  void addBody(RigidBody *bodyin, const char parentName_data[],
               const int parentName_size[2], CollisionSet &iobj_0,
               rigidBodyJoint &iobj_1, RigidBody &iobj_2);
  double findBodyIndexByName(const array<char, 2U> &bodyname);
  double findBodyIndexByJointName(const char jointname_data[],
                                  const int jointname_size[2]);
  void get_JointPositionLimits(array<double, 2U> &limits);
  void efficientFKAndJacobianForIK(const array<double, 1U> &qv, double bid1,
                                   double bid2, double T_data[], int T_size[2],
                                   array<double, 2U> &Jac);
  void kinematicPathInternal(RigidBody *body1, RigidBody *body2,
                             array<double, 2U> &indices);
  void ancestorIndices(RigidBody *body, array<double, 2U> &indices);
  void kinematicPath(const char body1Name_data[], const int body1Name_size[2],
                     const char body2Name_data[], const int body2Name_size[2],
                     array<double, 2U> &indices);
  boolean_T matlabCodegenIsDeleted;
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

class b_RigidBodyTree {
public:
  b_RigidBodyTree *init();
  void matlabCodegenDestructor();
  ~b_RigidBodyTree();
  b_RigidBodyTree();
  boolean_T matlabCodegenIsDeleted;
  b_RigidBody Base;
  FastVisualizationHelper b_FastVisualizationHelper;
  b_RigidBody *Bodies[1];
  b_RigidBody _pobj0;
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

// Function Declarations
void binary_expand_op_29(boolean_T in1[9], const double in2[9],
                         const coder::array<double, 2U> &in3);

void binary_expand_op_30(boolean_T in1[9], const double in2[9],
                         const coder::array<double, 2U> &in3);

#endif
//
// File trailer for RigidBodyTree.h
//
// [EOF]
//
