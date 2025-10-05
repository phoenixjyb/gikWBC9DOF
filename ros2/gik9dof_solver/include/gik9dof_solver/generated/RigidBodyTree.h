//
// RigidBodyTree.h
//
// Code generation for function 'RigidBodyTree'
//

#ifndef RIGIDBODYTREE_H
#define RIGIDBODYTREE_H

// Include files
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
  b_RigidBodyTree *init();
  double findBodyIndexByName(const char bodyname_data[],
                             const int bodyname_size[2]);
  double findBodyIndexByJointName(const char jointname_data[],
                                  const int jointname_size[2]);
  void addBody(RigidBody *bodyin, const char parentName_data[],
               const int parentName_size[2], CollisionSet &iobj_0,
               rigidBodyJoint &iobj_1, RigidBody &iobj_2);
  void get_JointPositionLimits(array<double, 2U> &limits);
  void efficientFKAndJacobianForIK(const array<double, 1U> &qv, double bid1,
                                   double bid2, double T_data[], int T_size[2],
                                   array<double, 2U> &Jac);
  void kinematicPathInternal(RigidBody *body1, RigidBody *body2,
                             array<double, 2U> &indices);
  void ancestorIndices(RigidBody *body, array<double, 2U> &indices);
  double validateInputBodyName(const char bodyname[17]);
  b_RigidBodyTree();
  ~b_RigidBodyTree();
  bool matlabCodegenIsDeleted;
  double NumBodies;
  RigidBody Base;
  FastVisualizationHelper b_FastVisualizationHelper;
  double Gravity[3];
  RigidBody *Bodies[12];
  double NumNonFixedBodies;
  double PositionNumber;
  double VelocityNumber;
  double PositionDoFMap[24];
  double VelocityDoFMap[24];
  CollisionSet _pobj0[3];
  rigidBodyJoint _pobj1[3];
  RigidBody _pobj2[12];
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

// Function Declarations
void binary_expand_op_10(bool in1[9], const double in2[9],
                         const coder::array<double, 2U> &in3);

void binary_expand_op_11(bool in1[9], const double in2[9],
                         const coder::array<double, 2U> &in3);

#endif
// End of code generation (RigidBodyTree.h)
