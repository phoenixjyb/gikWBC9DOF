//
// File: buildRobotForCodegen.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 09-Oct-2025 12:02:50
//

// Include Files
#include "buildRobotForCodegen.h"
#include "CollisionSet.h"
#include "GIKSolver.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "rigidBody1.h"
#include "rigidBodyJoint.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// BUILDROBOTFORCODEGEN Build 9-DOF mobile manipulator procedurally for code
// generation
//    This function constructs the rigidBodyTree without any file I/O,
//    making it compatible with MATLAB Coder for C++ code generation.
//
//    Robot structure: 3 base DOFs (x, y, theta) + 6 arm DOFs
//    All inertial, visual, and joint parameters are hardcoded from URDF.
//
//
// Arguments    : GIKSolver *aInstancePtr
//                coder::robotics::manip::internal::CollisionSet &iobj_0
//                coder::rigidBodyJoint &iobj_1
//                coder::robotics::manip::internal::RigidBody &iobj_2
//                coder::rigidBodyTree &iobj_3
// Return Type  : coder::rigidBodyTree *
//
namespace gik9dof {
namespace codegen_inuse {
coder::rigidBodyTree *
buildRobotForCodegen(GIKSolver *aInstancePtr,
                     coder::robotics::manip::internal::CollisionSet &iobj_0,
                     coder::rigidBodyJoint &iobj_1,
                     coder::robotics::manip::internal::RigidBody &iobj_2,
                     coder::rigidBodyTree &iobj_3)
{
  coder::rigidBody armBase;
  coder::rigidBody armLink1;
  coder::rigidBody armLink2;
  coder::rigidBody armLink3;
  coder::rigidBody armLink4;
  coder::rigidBody armLink5;
  coder::rigidBody armLink6;
  coder::rigidBody baseX;
  coder::rigidBody baseY;
  coder::rigidBody chassis;
  coder::rigidBody gripper;
  coder::rigidBodyJoint lobj_24[22];
  coder::rigidBodyJoint jntArm1;
  coder::rigidBodyJoint jntArm2;
  coder::rigidBodyJoint jntArm3;
  coder::rigidBodyJoint jntArm4;
  coder::rigidBodyJoint jntArm5;
  coder::rigidBodyJoint jntArm6;
  coder::rigidBodyJoint jntArmMount;
  coder::rigidBodyJoint jntGripper;
  coder::rigidBodyJoint jntTheta;
  coder::rigidBodyJoint jntX;
  coder::rigidBodyJoint jntY;
  coder::rigidBodyTree *robot;
  coder::robotics::manip::internal::CollisionSet lobj_23[11];
  coder::robotics::manip::internal::RigidBody lobj_25[11];
  coder::robotics::manip::internal::RigidBodyTree lobj_22[11];
  char parentName_data[200];
  for (int i{0}; i < 11; i++) {
    lobj_25[i]._pobj0.matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 11; i++) {
    lobj_23[i].matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 11; i++) {
    lobj_22[i]._pobj0.CollisionsInternal.matlabCodegenIsDeleted = true;
    lobj_22[i].Base.CollisionsInternal.matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 11; i++) {
    lobj_25[i].matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 11; i++) {
    lobj_22[i]._pobj0.matlabCodegenIsDeleted = true;
    lobj_22[i].Base.matlabCodegenIsDeleted = true;
    lobj_22[i].matlabCodegenIsDeleted = true;
  }
  int parentName_size[2];
  gripper.matlabCodegenIsDeleted = true;
  armLink6.matlabCodegenIsDeleted = true;
  armLink5.matlabCodegenIsDeleted = true;
  armLink4.matlabCodegenIsDeleted = true;
  armLink3.matlabCodegenIsDeleted = true;
  armLink2.matlabCodegenIsDeleted = true;
  armLink1.matlabCodegenIsDeleted = true;
  armBase.matlabCodegenIsDeleted = true;
  chassis.matlabCodegenIsDeleted = true;
  baseY.matlabCodegenIsDeleted = true;
  baseX.matlabCodegenIsDeleted = true;
  //  Create rigid body tree with column data format (required for GIK)
  //  MaxNumBodies is required for code generation (base + 10 bodies = 11)
  robot = iobj_3.init(aInstancePtr);
  robot->TreeInternal.Gravity[0] = 0.0;
  robot->TreeInternal.Gravity[1] = 0.0;
  robot->TreeInternal.Gravity[2] = -9.81;
  //  BASE JOINTS (3 DOF planar base: x, y, theta)
  //  Joint 1: Prismatic X (joint_x)
  baseX.init(aInstancePtr, lobj_22[0], lobj_23[0], lobj_24[0], lobj_25[0]);
  baseX.set_Mass();
  baseX.set_CenterOfMass();
  baseX.set_Inertia();
  jntX.b_init();
  jntX.setFixedTransform();
  jntX.set_JointAxis();
  jntX.set_PositionLimits();
  jntX.set_HomePosition();
  baseX.BodyInternal->JointInternal = jntX.copy(lobj_24[1]);
  robot->get_BaseName(parentName_data, parentName_size);
  robot->TreeInternal.addBody(baseX.BodyInternal, parentName_data,
                              parentName_size, (&iobj_0)[0], (&iobj_1)[0],
                              (&iobj_2)[0]);
  //  Joint 2: Prismatic Y (joint_y)
  baseY.b_init(aInstancePtr, lobj_22[1], lobj_23[1], lobj_24[2], lobj_25[1]);
  baseY.set_Mass();
  baseY.set_CenterOfMass();
  baseY.set_Inertia();
  jntY.c_init();
  jntY.setFixedTransform();
  jntY.b_set_JointAxis();
  jntY.set_PositionLimits();
  jntY.set_HomePosition();
  baseY.BodyInternal->JointInternal = jntY.copy(lobj_24[3]);
  robot->addBody(baseY, (&iobj_0)[2], (&iobj_1)[2], (&iobj_2)[1]);
  //  Joint 3: Revolute Theta (joint_theta)
  chassis.c_init(aInstancePtr, lobj_22[2], lobj_23[2], lobj_24[4], lobj_25[2]);
  chassis.b_set_Mass();
  chassis.b_set_CenterOfMass();
  chassis.b_set_Inertia();
  jntTheta.d_init();
  jntTheta.setFixedTransform();
  jntTheta.c_set_JointAxis();
  jntTheta.set_PositionLimits(aInstancePtr);
  jntTheta.set_HomePosition();
  chassis.BodyInternal->JointInternal = jntTheta.copy(lobj_24[5]);
  robot->b_addBody(chassis, (&iobj_0)[4], (&iobj_1)[4], (&iobj_2)[2]);
  //  ARM BASE LINK (fixed mount on chassis)
  armBase.d_init(aInstancePtr, lobj_22[3], lobj_23[3], lobj_24[6], lobj_25[3]);
  armBase.c_set_Mass();
  armBase.c_set_CenterOfMass();
  armBase.c_set_Inertia();
  jntArmMount.e_init();
  //  Origin: rpy="0 0 -1.5708" xyz="0.15995 0 0.9465"
  jntArmMount.b_setFixedTransform();
  armBase.BodyInternal->JointInternal = jntArmMount.copy(lobj_24[7]);
  robot->c_addBody(armBase, (&iobj_0)[6], (&iobj_1)[6], (&iobj_2)[3]);
  //  ARM JOINT 1 (revolute, Y-axis)
  armLink1.e_init(aInstancePtr, lobj_22[4], lobj_23[4], lobj_24[8], lobj_25[4]);
  armLink1.d_set_Mass();
  armLink1.d_set_CenterOfMass();
  armLink1.d_set_Inertia();
  jntArm1.f_init();
  jntArm1.c_setFixedTransform();
  jntArm1.b_set_JointAxis();
  jntArm1.b_set_PositionLimits();
  jntArm1.set_HomePosition();
  armLink1.BodyInternal->JointInternal = jntArm1.copy(lobj_24[9]);
  robot->d_addBody(armLink1, (&iobj_0)[8], (&iobj_1)[8], (&iobj_2)[4]);
  //  ARM JOINT 2 (revolute, -Z-axis)
  armLink2.f_init(aInstancePtr, lobj_22[5], lobj_23[5], lobj_24[10],
                  lobj_25[5]);
  armLink2.e_set_Mass();
  armLink2.e_set_CenterOfMass();
  armLink2.e_set_Inertia();
  jntArm2.g_init();
  jntArm2.d_setFixedTransform();
  jntArm2.d_set_JointAxis();
  jntArm2.c_set_PositionLimits();
  jntArm2.set_HomePosition();
  armLink2.BodyInternal->JointInternal = jntArm2.copy(lobj_24[11]);
  robot->e_addBody(armLink2, (&iobj_0)[10], (&iobj_1)[10], (&iobj_2)[5]);
  //  ARM JOINT 3 (revolute, -Z-axis)
  armLink3.g_init(aInstancePtr, lobj_22[6], lobj_23[6], lobj_24[12],
                  lobj_25[6]);
  armLink3.f_set_Mass();
  armLink3.f_set_CenterOfMass();
  armLink3.f_set_Inertia();
  jntArm3.h_init();
  jntArm3.e_setFixedTransform();
  jntArm3.d_set_JointAxis();
  jntArm3.d_set_PositionLimits();
  jntArm3.set_HomePosition();
  armLink3.BodyInternal->JointInternal = jntArm3.copy(lobj_24[13]);
  robot->f_addBody(armLink3, (&iobj_0)[12], (&iobj_1)[12], (&iobj_2)[6]);
  //  ARM JOINT 4 (revolute, X-axis)
  armLink4.h_init(aInstancePtr, lobj_22[7], lobj_23[7], lobj_24[14],
                  lobj_25[7]);
  armLink4.g_set_Mass();
  armLink4.g_set_CenterOfMass();
  armLink4.g_set_Inertia();
  jntArm4.i_init();
  jntArm4.f_setFixedTransform();
  jntArm4.set_JointAxis();
  jntArm4.b_set_PositionLimits();
  jntArm4.set_HomePosition();
  armLink4.BodyInternal->JointInternal = jntArm4.copy(lobj_24[15]);
  robot->g_addBody(armLink4, (&iobj_0)[14], (&iobj_1)[14], (&iobj_2)[7]);
  //  ARM JOINT 5 (revolute, Y-axis)
  armLink5.i_init(aInstancePtr, lobj_22[8], lobj_23[8], lobj_24[16],
                  lobj_25[8]);
  armLink5.h_set_Mass();
  armLink5.h_set_CenterOfMass();
  armLink5.h_set_Inertia();
  jntArm5.j_init();
  jntArm5.g_setFixedTransform();
  jntArm5.b_set_JointAxis();
  jntArm5.e_set_PositionLimits();
  jntArm5.set_HomePosition();
  armLink5.BodyInternal->JointInternal = jntArm5.copy(lobj_24[17]);
  robot->h_addBody(armLink5, (&iobj_0)[16], (&iobj_1)[16], (&iobj_2)[8]);
  //  ARM JOINT 6 (revolute, X-axis)
  armLink6.j_init(aInstancePtr, lobj_22[9], lobj_23[9], lobj_24[18],
                  lobj_25[9]);
  armLink6.i_set_Mass();
  armLink6.i_set_CenterOfMass();
  armLink6.i_set_Inertia();
  jntArm6.k_init();
  jntArm6.h_setFixedTransform();
  jntArm6.set_JointAxis();
  jntArm6.b_set_PositionLimits();
  jntArm6.set_HomePosition();
  armLink6.BodyInternal->JointInternal = jntArm6.copy(lobj_24[19]);
  robot->i_addBody(armLink6, (&iobj_0)[18], (&iobj_1)[18], (&iobj_2)[9]);
  //  END EFFECTOR (fixed gripper link)
  gripper.k_init(aInstancePtr, lobj_22[10], lobj_23[10], lobj_24[20],
                 lobj_25[10]);
  gripper.j_set_Mass();
  gripper.j_set_CenterOfMass();
  gripper.j_set_Inertia();
  jntGripper.l_init();
  jntGripper.i_setFixedTransform();
  gripper.BodyInternal->JointInternal = jntGripper.copy(lobj_24[21]);
  robot->j_addBody(gripper, (&iobj_0)[20], (&iobj_1)[20], (&iobj_2)[10]);
  baseX.matlabCodegenDestructor();
  baseY.matlabCodegenDestructor();
  chassis.matlabCodegenDestructor();
  armBase.matlabCodegenDestructor();
  armLink1.matlabCodegenDestructor();
  armLink2.matlabCodegenDestructor();
  armLink3.matlabCodegenDestructor();
  armLink4.matlabCodegenDestructor();
  armLink5.matlabCodegenDestructor();
  armLink6.matlabCodegenDestructor();
  gripper.matlabCodegenDestructor();
  for (int i{0}; i < 11; i++) {
    lobj_22[i].matlabCodegenDestructor();
  }
  for (int i{0}; i < 11; i++) {
    lobj_22[i].Base.matlabCodegenDestructor();
  }
  for (int i{0}; i < 11; i++) {
    lobj_22[i]._pobj0.matlabCodegenDestructor();
  }
  for (int i{0}; i < 11; i++) {
    lobj_25[i].matlabCodegenDestructor();
  }
  for (int i{0}; i < 11; i++) {
    lobj_22[i].Base.CollisionsInternal.matlabCodegenDestructor();
  }
  for (int i{0}; i < 11; i++) {
    lobj_22[i]._pobj0.CollisionsInternal.matlabCodegenDestructor();
  }
  for (int i{0}; i < 11; i++) {
    lobj_23[i].matlabCodegenDestructor();
  }
  for (int i{0}; i < 11; i++) {
    lobj_25[i]._pobj0.matlabCodegenDestructor();
  }
  return robot;
}

} // namespace codegen_inuse
} // namespace gik9dof

//
// File trailer for buildRobotForCodegen.cpp
//
// [EOF]
//
