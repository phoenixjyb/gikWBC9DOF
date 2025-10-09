//
// File: buildRobotForCodegen.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "buildRobotForCodegen.h"
#include "CharacterVector.h"
#include "CollisionSet.h"
#include "GIKSolver.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "rigidBody1.h"
#include "rigidBodyJoint.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"
#include "solveGIKStepWrapper_data.h"
#include <algorithm>
#include <cstring>
#include <emmintrin.h>

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
  static const double b_inputTform[16]{1.0,        0.0,    0.0, 0.0, 0.0, 1.0,
                                       0.0,        0.0,    0.0, 0.0, 1.0, 0.0,
                                       -0.0011149, 0.0446, 0.0, 1.0};
  static const double c_inputTform[16]{1.0, 0.0,    0.0, 0.0, 0.0, 1.0,
                                       0.0, 0.0,    0.0, 0.0, 1.0, 0.0,
                                       0.0, 0.1061, 0.0, 1.0};
  static const double d_inputTform[16]{1.0,      0.0,      0.0, 0.0, 0.0, 1.0,
                                       0.0,      0.0,      0.0, 0.0, 1.0, 0.0,
                                       -0.34928, 0.019998, 0.0, 1.0};
  static const double e_inputTform[16]{1.0,     0.0,      0.0, 0.0, 0.0, 1.0,
                                       0.0,     0.0,      0.0, 0.0, 1.0, 0.0,
                                       0.02735, 0.069767, 0.0, 1.0};
  static const double f_inputTform[16]{1.0,    0.0,         0.0, 0.0, 0.0, 1.0,
                                       0.0,    0.0,         0.0, 0.0, 1.0, 0.0,
                                       0.2463, -0.00049894, 0.0, 1.0};
  static const double g_inputTform[16]{1.0,      0.0,        0.0, 0.0, 0.0, 1.0,
                                       0.0,      0.0,        0.0, 0.0, 1.0, 0.0,
                                       0.058249, 0.00050025, 0.0, 1.0};
  static const double h_inputTform[16]{1.0,    0.0, 0.0, 0.0, 0.0, 1.0,
                                       0.0,    0.0, 0.0, 0.0, 1.0, 0.0,
                                       0.1039, 0.0, 0.0, 1.0};
  static const double inputTform[16]{-3.6732051033465739E-6,
                                     -0.99999999999325373,
                                     0.0,
                                     0.0,
                                     0.99999999999325373,
                                     -3.6732051033465739E-6,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     1.0,
                                     0.0,
                                     0.15995,
                                     0.0,
                                     0.9465,
                                     1.0};
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
  coder::robotics::manip::internal::CharacterVector obj;
  coder::robotics::manip::internal::CollisionSet lobj_23[11];
  coder::robotics::manip::internal::RigidBody lobj_25[11];
  coder::robotics::manip::internal::RigidBodyTree lobj_22[11];
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int obj_size[2];
  int loop_ub;
  char obj_data[200];
  signed char b_I[16];
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
  robot = &iobj_3;
  iobj_3.TreeInternal.init(aInstancePtr);
  iobj_3.TreeInternal.Base.CollisionsInternal = iobj_3._pobj0.init(10.0);
  iobj_3.matlabCodegenIsDeleted = false;
  iobj_3.TreeInternal.Gravity[0] = 0.0;
  iobj_3.TreeInternal.Gravity[1] = 0.0;
  iobj_3.TreeInternal.Gravity[2] = -9.81;
  //  BASE JOINTS (3 DOF planar base: x, y, theta)
  //  Joint 1: Prismatic X (joint_x)
  baseX.init(aInstancePtr, lobj_22[0], lobj_23[0], lobj_24[0], lobj_25[0]);
  baseX.set_Mass();
  baseX.set_CenterOfMass();
  baseX.set_Inertia();
  jntX.b_init();
  for (int i{0}; i < 16; i++) {
    jntX.JointToParentTransform[i] = iv[i];
    jntX.ChildToJointTransform[i] = 0.0;
  }
  jntX.ChildToJointTransform[0] = 1.0;
  jntX.ChildToJointTransform[5] = 1.0;
  jntX.ChildToJointTransform[10] = 1.0;
  jntX.ChildToJointTransform[15] = 1.0;
  jntX.set_JointAxis();
  jntX.set_PositionLimits();
  mass = jntX.PositionNumber;
  if (mass < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(mass);
  }
  if (loop_ub - 1 >= 0) {
    std::memset(&jntX.HomePositionInternal[0], 0,
                static_cast<unsigned int>(loop_ub) * sizeof(double));
  }
  baseX.BodyInternal->JointInternal = jntX.copy(lobj_24[1]);
  obj = iobj_3.TreeInternal.Base.NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  iobj_3.TreeInternal.addBody(baseX.BodyInternal, obj_data, obj_size,
                              (&(&iobj_0)[0])[0], (&(&iobj_1)[0])[0],
                              (&iobj_2)[0]);
  //  Joint 2: Prismatic Y (joint_y)
  baseY.b_init(aInstancePtr, lobj_22[1], lobj_23[1], lobj_24[2], lobj_25[1]);
  baseY.set_Mass();
  baseY.set_CenterOfMass();
  baseY.set_Inertia();
  jntY.c_init();
  for (int i{0}; i < 16; i++) {
    jntY.JointToParentTransform[i] = iv[i];
    jntY.ChildToJointTransform[i] = 0.0;
  }
  jntY.ChildToJointTransform[0] = 1.0;
  jntY.ChildToJointTransform[5] = 1.0;
  jntY.ChildToJointTransform[10] = 1.0;
  jntY.ChildToJointTransform[15] = 1.0;
  jntY.b_set_JointAxis();
  jntY.set_PositionLimits();
  mass = jntY.PositionNumber;
  if (mass < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(mass);
  }
  if (loop_ub - 1 >= 0) {
    std::memset(&jntY.HomePositionInternal[0], 0,
                static_cast<unsigned int>(loop_ub) * sizeof(double));
  }
  baseY.BodyInternal->JointInternal = jntY.copy(lobj_24[3]);
  iobj_3.addBody(baseY, (&iobj_0)[2], (&iobj_1)[2], (&iobj_2)[1]);
  //  Joint 3: Revolute Theta (joint_theta)
  chassis.c_init(aInstancePtr, lobj_22[2], lobj_23[2], lobj_24[4], lobj_25[2]);
  chassis.BodyInternal->MassInternal = 50.96231322;
  mass = chassis.BodyInternal->MassInternal;
  com_idx_0 = chassis.BodyInternal->CenterOfMassInternal[0];
  com_idx_1 = chassis.BodyInternal->CenterOfMassInternal[1];
  com_idx_2 = chassis.BodyInternal->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = chassis.BodyInternal->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    chassis.BodyInternal->SpatialInertia[6 * i] = inertia[3 * i];
    chassis.BodyInternal->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    chassis.BodyInternal->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  __m128d r;
  __m128d r1;
  r = _mm_loadu_pd(&sc[0]);
  r1 = _mm_set1_pd(mass);
  _mm_storeu_pd(&chassis.BodyInternal->SpatialInertia[18], _mm_mul_pd(r1, r));
  chassis.BodyInternal->SpatialInertia[20] = mass * -com_idx_1;
  r = _mm_loadu_pd(&sc[3]);
  _mm_storeu_pd(&chassis.BodyInternal->SpatialInertia[24], _mm_mul_pd(r1, r));
  chassis.BodyInternal->SpatialInertia[26] = mass * com_idx_0;
  r = _mm_loadu_pd(&sc[6]);
  _mm_storeu_pd(&chassis.BodyInternal->SpatialInertia[30], _mm_mul_pd(r1, r));
  chassis.BodyInternal->SpatialInertia[32] = mass * 0.0;
  for (int i{0}; i < 3; i++) {
    chassis.BodyInternal->SpatialInertia[6 * i + 3] = mass * sc[i];
    chassis.BodyInternal->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    chassis.BodyInternal->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    loop_ub = 6 * (i + 3);
    chassis.BodyInternal->SpatialInertia[loop_ub + 3] =
        mass * static_cast<double>(iv1[3 * i]);
    chassis.BodyInternal->SpatialInertia[loop_ub + 4] =
        mass * static_cast<double>(iv1[3 * i + 1]);
    chassis.BodyInternal->SpatialInertia[loop_ub + 5] =
        mass * static_cast<double>(iv1[3 * i + 2]);
  }
  chassis.b_set_CenterOfMass();
  chassis.b_set_Inertia();
  jntTheta.d_init();
  for (int i{0}; i < 16; i++) {
    jntTheta.JointToParentTransform[i] = iv[i];
    jntTheta.ChildToJointTransform[i] = 0.0;
  }
  jntTheta.ChildToJointTransform[0] = 1.0;
  jntTheta.ChildToJointTransform[5] = 1.0;
  jntTheta.ChildToJointTransform[10] = 1.0;
  jntTheta.ChildToJointTransform[15] = 1.0;
  jntTheta.c_set_JointAxis();
  jntTheta.set_PositionLimits(aInstancePtr);
  mass = jntTheta.PositionNumber;
  if (mass < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(mass);
  }
  if (loop_ub - 1 >= 0) {
    std::memset(&jntTheta.HomePositionInternal[0], 0,
                static_cast<unsigned int>(loop_ub) * sizeof(double));
  }
  chassis.BodyInternal->JointInternal = jntTheta.copy(lobj_24[5]);
  iobj_3.b_addBody(chassis, (&iobj_0)[4], (&iobj_1)[4], (&iobj_2)[2]);
  //  ARM BASE LINK (fixed mount on chassis)
  armBase.d_init(aInstancePtr, lobj_22[3], lobj_23[3], lobj_24[6], lobj_25[3]);
  armBase.b_set_Mass();
  armBase.c_set_CenterOfMass();
  armBase.c_set_Inertia();
  jntArmMount.e_init();
  //  Origin: rpy="0 0 -1.5708" xyz="0.15995 0 0.9465"
  for (int i{0}; i < 16; i++) {
    jntArmMount.JointToParentTransform[i] = inputTform[i];
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[5] = 1;
  b_I[10] = 1;
  b_I[15] = 1;
  for (int i{0}; i < 16; i++) {
    jntArmMount.ChildToJointTransform[i] = b_I[i];
  }
  armBase.BodyInternal->JointInternal = jntArmMount.copy(lobj_24[7]);
  iobj_3.c_addBody(armBase, (&iobj_0)[6], (&iobj_1)[6], (&iobj_2)[3]);
  //  ARM JOINT 1 (revolute, Y-axis)
  armLink1.e_init(aInstancePtr, lobj_22[4], lobj_23[4], lobj_24[8], lobj_25[4]);
  armLink1.c_set_Mass();
  armLink1.d_set_CenterOfMass();
  armLink1.d_set_Inertia();
  jntArm1.f_init();
  for (int i{0}; i < 16; i++) {
    jntArm1.JointToParentTransform[i] = b_inputTform[i];
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[5] = 1;
  b_I[10] = 1;
  b_I[15] = 1;
  for (int i{0}; i < 16; i++) {
    jntArm1.ChildToJointTransform[i] = b_I[i];
  }
  jntArm1.b_set_JointAxis();
  jntArm1.b_set_PositionLimits();
  mass = jntArm1.PositionNumber;
  if (mass < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(mass);
  }
  if (loop_ub - 1 >= 0) {
    std::memset(&jntArm1.HomePositionInternal[0], 0,
                static_cast<unsigned int>(loop_ub) * sizeof(double));
  }
  armLink1.BodyInternal->JointInternal = jntArm1.copy(lobj_24[9]);
  iobj_3.d_addBody(armLink1, (&iobj_0)[8], (&iobj_1)[8], (&iobj_2)[4]);
  //  ARM JOINT 2 (revolute, -Z-axis)
  armLink2.f_init(aInstancePtr, lobj_22[5], lobj_23[5], lobj_24[10],
                  lobj_25[5]);
  armLink2.d_set_Mass();
  armLink2.e_set_CenterOfMass();
  armLink2.e_set_Inertia();
  jntArm2.g_init();
  for (int i{0}; i < 16; i++) {
    jntArm2.JointToParentTransform[i] = c_inputTform[i];
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[5] = 1;
  b_I[10] = 1;
  b_I[15] = 1;
  for (int i{0}; i < 16; i++) {
    jntArm2.ChildToJointTransform[i] = b_I[i];
  }
  jntArm2.d_set_JointAxis();
  jntArm2.c_set_PositionLimits();
  mass = jntArm2.PositionNumber;
  if (mass < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(mass);
  }
  if (loop_ub - 1 >= 0) {
    std::memset(&jntArm2.HomePositionInternal[0], 0,
                static_cast<unsigned int>(loop_ub) * sizeof(double));
  }
  armLink2.BodyInternal->JointInternal = jntArm2.copy(lobj_24[11]);
  iobj_3.e_addBody(armLink2, (&iobj_0)[10], (&iobj_1)[10], (&iobj_2)[5]);
  //  ARM JOINT 3 (revolute, -Z-axis)
  armLink3.g_init(aInstancePtr, lobj_22[6], lobj_23[6], lobj_24[12],
                  lobj_25[6]);
  armLink3.e_set_Mass();
  armLink3.f_set_CenterOfMass();
  armLink3.f_set_Inertia();
  jntArm3.h_init();
  for (int i{0}; i < 16; i++) {
    jntArm3.JointToParentTransform[i] = d_inputTform[i];
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[5] = 1;
  b_I[10] = 1;
  b_I[15] = 1;
  for (int i{0}; i < 16; i++) {
    jntArm3.ChildToJointTransform[i] = b_I[i];
  }
  jntArm3.d_set_JointAxis();
  jntArm3.d_set_PositionLimits();
  mass = jntArm3.PositionNumber;
  if (mass < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(mass);
  }
  if (loop_ub - 1 >= 0) {
    std::memset(&jntArm3.HomePositionInternal[0], 0,
                static_cast<unsigned int>(loop_ub) * sizeof(double));
  }
  armLink3.BodyInternal->JointInternal = jntArm3.copy(lobj_24[13]);
  iobj_3.f_addBody(armLink3, (&iobj_0)[12], (&iobj_1)[12], (&iobj_2)[6]);
  //  ARM JOINT 4 (revolute, X-axis)
  armLink4.h_init(aInstancePtr, lobj_22[7], lobj_23[7], lobj_24[14],
                  lobj_25[7]);
  armLink4.f_set_Mass();
  armLink4.g_set_CenterOfMass();
  armLink4.g_set_Inertia();
  jntArm4.i_init();
  for (int i{0}; i < 16; i++) {
    jntArm4.JointToParentTransform[i] = e_inputTform[i];
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[5] = 1;
  b_I[10] = 1;
  b_I[15] = 1;
  for (int i{0}; i < 16; i++) {
    jntArm4.ChildToJointTransform[i] = b_I[i];
  }
  jntArm4.set_JointAxis();
  jntArm4.b_set_PositionLimits();
  mass = jntArm4.PositionNumber;
  if (mass < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(mass);
  }
  if (loop_ub - 1 >= 0) {
    std::memset(&jntArm4.HomePositionInternal[0], 0,
                static_cast<unsigned int>(loop_ub) * sizeof(double));
  }
  armLink4.BodyInternal->JointInternal = jntArm4.copy(lobj_24[15]);
  iobj_3.g_addBody(armLink4, (&iobj_0)[14], (&iobj_1)[14], (&iobj_2)[7]);
  //  ARM JOINT 5 (revolute, Y-axis)
  armLink5.i_init(aInstancePtr, lobj_22[8], lobj_23[8], lobj_24[16],
                  lobj_25[8]);
  armLink5.g_set_Mass();
  armLink5.h_set_CenterOfMass();
  armLink5.h_set_Inertia();
  jntArm5.j_init();
  for (int i{0}; i < 16; i++) {
    jntArm5.JointToParentTransform[i] = f_inputTform[i];
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[5] = 1;
  b_I[10] = 1;
  b_I[15] = 1;
  for (int i{0}; i < 16; i++) {
    jntArm5.ChildToJointTransform[i] = b_I[i];
  }
  jntArm5.b_set_JointAxis();
  jntArm5.e_set_PositionLimits();
  mass = jntArm5.PositionNumber;
  if (mass < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(mass);
  }
  if (loop_ub - 1 >= 0) {
    std::memset(&jntArm5.HomePositionInternal[0], 0,
                static_cast<unsigned int>(loop_ub) * sizeof(double));
  }
  armLink5.BodyInternal->JointInternal = jntArm5.copy(lobj_24[17]);
  iobj_3.h_addBody(armLink5, (&iobj_0)[16], (&iobj_1)[16], (&iobj_2)[8]);
  //  ARM JOINT 6 (revolute, X-axis)
  armLink6.j_init(aInstancePtr, lobj_22[9], lobj_23[9], lobj_24[18],
                  lobj_25[9]);
  armLink6.h_set_Mass();
  armLink6.i_set_CenterOfMass();
  armLink6.i_set_Inertia();
  jntArm6.k_init();
  for (int i{0}; i < 16; i++) {
    jntArm6.JointToParentTransform[i] = g_inputTform[i];
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[5] = 1;
  b_I[10] = 1;
  b_I[15] = 1;
  for (int i{0}; i < 16; i++) {
    jntArm6.ChildToJointTransform[i] = b_I[i];
  }
  jntArm6.set_JointAxis();
  jntArm6.b_set_PositionLimits();
  mass = jntArm6.PositionNumber;
  if (mass < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(mass);
  }
  if (loop_ub - 1 >= 0) {
    std::memset(&jntArm6.HomePositionInternal[0], 0,
                static_cast<unsigned int>(loop_ub) * sizeof(double));
  }
  armLink6.BodyInternal->JointInternal = jntArm6.copy(lobj_24[19]);
  iobj_3.i_addBody(armLink6, (&iobj_0)[18], (&iobj_1)[18], (&iobj_2)[9]);
  //  END EFFECTOR (fixed gripper link)
  gripper.k_init(aInstancePtr, lobj_22[10], lobj_23[10], lobj_24[20],
                 lobj_25[10]);
  gripper.i_set_Mass();
  gripper.j_set_CenterOfMass();
  gripper.j_set_Inertia();
  jntGripper.l_init();
  for (int i{0}; i < 16; i++) {
    jntGripper.JointToParentTransform[i] = h_inputTform[i];
    b_I[i] = 0;
  }
  b_I[0] = 1;
  b_I[5] = 1;
  b_I[10] = 1;
  b_I[15] = 1;
  for (int i{0}; i < 16; i++) {
    jntGripper.ChildToJointTransform[i] = b_I[i];
  }
  gripper.BodyInternal->JointInternal = jntGripper.copy(lobj_24[21]);
  iobj_3.j_addBody(gripper, (&iobj_0)[20], (&iobj_1)[20], (&iobj_2)[10]);
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
