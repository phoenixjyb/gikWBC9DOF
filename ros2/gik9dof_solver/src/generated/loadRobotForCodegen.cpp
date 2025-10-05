//
// loadRobotForCodegen.cpp
//
// Code generation for function 'loadRobotForCodegen'
//

// Include files
#include "loadRobotForCodegen.h"
#include "CollisionSet.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "buildRobotForCodegen.h"
#include "gik9dof_codegen_followTrajectory_data.h"
#include "rigidBody1.h"
#include "rigidBodyJoint.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"
#include <algorithm>

// Variable Definitions
static coder::robotics::manip::internal::RigidBody gobj_7[11];

static coder::robotics::manip::internal::CollisionSet gobj_5[22];

static bool robotPersist_not_empty;

// Function Definitions
namespace gik9dof {
namespace codegen {
coder::rigidBodyTree *loadRobotForCodegen()
{
  static coder::rigidBodyJoint gobj_6[22];
  static coder::rigidBodyTree *robotPersist;
  static const double dv[16]{6.123233995736766E-17,
                             -1.0,
                             -0.0,
                             0.0,
                             1.0,
                             6.123233995736766E-17,
                             0.0,
                             0.0,
                             0.0,
                             -0.0,
                             1.0,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             1.0};
  coder::rigidBody b_body;
  coder::rigidBody body;
  coder::rigidBody c_body;
  coder::rigidBody d_body;
  coder::rigidBody e_body;
  coder::rigidBody f_body;
  coder::rigidBody g_body;
  coder::rigidBodyJoint b_lobj_4[2];
  coder::rigidBodyJoint c_lobj_4[2];
  coder::rigidBodyJoint d_lobj_4[2];
  coder::rigidBodyJoint e_lobj_4[2];
  coder::rigidBodyJoint f_lobj_4[2];
  coder::rigidBodyJoint g_lobj_4[2];
  coder::rigidBodyJoint lobj_4[2];
  coder::rigidBodyJoint b_joint;
  coder::rigidBodyJoint c_joint;
  coder::rigidBodyJoint d_joint;
  coder::rigidBodyJoint e_joint;
  coder::rigidBodyJoint f_joint;
  coder::rigidBodyJoint g_joint;
  coder::rigidBodyJoint joint;
  coder::rigidBodyTree *b_robot;
  coder::robotics::manip::internal::CollisionSet b_lobj_3;
  coder::robotics::manip::internal::CollisionSet c_lobj_3;
  coder::robotics::manip::internal::CollisionSet d_lobj_3;
  coder::robotics::manip::internal::CollisionSet e_lobj_3;
  coder::robotics::manip::internal::CollisionSet f_lobj_3;
  coder::robotics::manip::internal::CollisionSet g_lobj_3;
  coder::robotics::manip::internal::CollisionSet lobj_3;
  coder::robotics::manip::internal::RigidBody b_lobj_5;
  coder::robotics::manip::internal::RigidBody c_lobj_5;
  coder::robotics::manip::internal::RigidBody d_lobj_5;
  coder::robotics::manip::internal::RigidBody e_lobj_5;
  coder::robotics::manip::internal::RigidBody f_lobj_5;
  coder::robotics::manip::internal::RigidBody g_lobj_5;
  coder::robotics::manip::internal::RigidBody lobj_5;
  coder::robotics::manip::internal::RigidBodyTree b_lobj_2;
  coder::robotics::manip::internal::RigidBodyTree c_lobj_2;
  coder::robotics::manip::internal::RigidBodyTree d_lobj_2;
  coder::robotics::manip::internal::RigidBodyTree e_lobj_2;
  coder::robotics::manip::internal::RigidBodyTree f_lobj_2;
  coder::robotics::manip::internal::RigidBodyTree g_lobj_2;
  coder::robotics::manip::internal::RigidBodyTree lobj_2;
  double tform[16];
  // LOADROBOTFORCODEGEN Build rigidBodyTree for C++ code generation.
  if (!robotPersist_not_empty) {
    // BUILDROBOTFORCODEGEN Construct the rigidBodyTree for code generation.
    b_robot = robot.init();
    b_robot->TreeInternal.Gravity[0] = 0.0;
    b_robot->TreeInternal.Gravity[1] = 0.0;
    b_robot->TreeInternal.Gravity[2] = -9.81;
    //  Planar base joints (X, Y, Yaw)
    lobj_5._pobj0.matlabCodegenIsDeleted = true;
    lobj_3.matlabCodegenIsDeleted = true;
    lobj_2._pobj0.CollisionsInternal.matlabCodegenIsDeleted = true;
    lobj_2.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
    lobj_5.matlabCodegenIsDeleted = true;
    lobj_2._pobj0.matlabCodegenIsDeleted = true;
    lobj_2.Base.matlabCodegenIsDeleted = true;
    lobj_2.matlabCodegenIsDeleted = true;
    body.matlabCodegenIsDeleted = true;
    body.init(lobj_2, lobj_3, lobj_4[0], lobj_5);
    joint.b_init();
    for (int i{0}; i < 16; i++) {
      tform[i] = iv[i];
    }
    tform[12] = 0.0;
    tform[13] = 0.0;
    tform[14] = 0.0;
    joint.setFixedTransform(tform);
    joint.set_JointAxis();
    joint.set_PositionLimits();
    body.BodyInternal->JointInternal = joint.copy(lobj_4[1]);
    b_robot->addBody(body, gobj_5[0], gobj_6[0], gobj_7[0]);
    body.matlabCodegenDestructor();
    lobj_2.matlabCodegenDestructor();
    lobj_2.Base.matlabCodegenDestructor();
    lobj_2._pobj0.matlabCodegenDestructor();
    lobj_5.matlabCodegenDestructor();
    lobj_2.Base.CollisionsInternal.matlabCodegenDestructor();
    lobj_2._pobj0.CollisionsInternal.matlabCodegenDestructor();
    lobj_3.matlabCodegenDestructor();
    lobj_5._pobj0.matlabCodegenDestructor();
    b_lobj_5._pobj0.matlabCodegenIsDeleted = true;
    b_lobj_3.matlabCodegenIsDeleted = true;
    b_lobj_2._pobj0.CollisionsInternal.matlabCodegenIsDeleted = true;
    b_lobj_2.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
    b_lobj_5.matlabCodegenIsDeleted = true;
    b_lobj_2._pobj0.matlabCodegenIsDeleted = true;
    b_lobj_2.Base.matlabCodegenIsDeleted = true;
    b_lobj_2.matlabCodegenIsDeleted = true;
    b_body.matlabCodegenIsDeleted = true;
    b_body.b_init(b_lobj_2, b_lobj_3, b_lobj_4[0], b_lobj_5);
    b_joint.c_init();
    for (int i{0}; i < 16; i++) {
      tform[i] = iv[i];
    }
    tform[12] = 0.0;
    tform[13] = 0.0;
    tform[14] = 0.0;
    b_joint.setFixedTransform(tform);
    b_joint.b_set_JointAxis();
    b_joint.set_PositionLimits();
    b_body.BodyInternal->JointInternal = b_joint.copy(b_lobj_4[1]);
    b_robot->b_addBody(b_body, gobj_5[2], gobj_6[2], gobj_7[1]);
    b_body.matlabCodegenDestructor();
    b_lobj_2.matlabCodegenDestructor();
    b_lobj_2.Base.matlabCodegenDestructor();
    b_lobj_2._pobj0.matlabCodegenDestructor();
    b_lobj_5.matlabCodegenDestructor();
    b_lobj_2.Base.CollisionsInternal.matlabCodegenDestructor();
    b_lobj_2._pobj0.CollisionsInternal.matlabCodegenDestructor();
    b_lobj_3.matlabCodegenDestructor();
    b_lobj_5._pobj0.matlabCodegenDestructor();
    c_lobj_5._pobj0.matlabCodegenIsDeleted = true;
    c_lobj_3.matlabCodegenIsDeleted = true;
    c_lobj_2._pobj0.CollisionsInternal.matlabCodegenIsDeleted = true;
    c_lobj_2.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
    c_lobj_5.matlabCodegenIsDeleted = true;
    c_lobj_2._pobj0.matlabCodegenIsDeleted = true;
    c_lobj_2.Base.matlabCodegenIsDeleted = true;
    c_lobj_2.matlabCodegenIsDeleted = true;
    c_body.matlabCodegenIsDeleted = true;
    c_body.c_init(c_lobj_2, c_lobj_3, c_lobj_4[0], c_lobj_5);
    c_joint.d_init();
    for (int i{0}; i < 16; i++) {
      tform[i] = iv[i];
    }
    tform[12] = 0.0;
    tform[13] = 0.0;
    tform[14] = 0.0;
    c_joint.setFixedTransform(tform);
    c_joint.c_set_JointAxis();
    c_joint.b_set_PositionLimits();
    c_body.BodyInternal->JointInternal = c_joint.copy(c_lobj_4[1]);
    b_robot->c_addBody(c_body, gobj_5[4], gobj_6[4], gobj_7[2]);
    c_body.matlabCodegenDestructor();
    c_lobj_2.matlabCodegenDestructor();
    c_lobj_2.Base.matlabCodegenDestructor();
    c_lobj_2._pobj0.matlabCodegenDestructor();
    c_lobj_5.matlabCodegenDestructor();
    c_lobj_2.Base.CollisionsInternal.matlabCodegenDestructor();
    c_lobj_2._pobj0.CollisionsInternal.matlabCodegenDestructor();
    c_lobj_3.matlabCodegenDestructor();
    c_lobj_5._pobj0.matlabCodegenDestructor();
    //  Fixed mount for the arm
    d_lobj_5._pobj0.matlabCodegenIsDeleted = true;
    d_lobj_3.matlabCodegenIsDeleted = true;
    d_lobj_2._pobj0.CollisionsInternal.matlabCodegenIsDeleted = true;
    d_lobj_2.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
    d_lobj_5.matlabCodegenIsDeleted = true;
    d_lobj_2._pobj0.matlabCodegenIsDeleted = true;
    d_lobj_2.Base.matlabCodegenIsDeleted = true;
    d_lobj_2.matlabCodegenIsDeleted = true;
    d_body.matlabCodegenIsDeleted = true;
    d_body.d_init(d_lobj_2, d_lobj_3, d_lobj_4[0], d_lobj_5);
    d_joint.e_init();
    std::copy(&dv[0], &dv[16], &tform[0]);
    tform[12] = 0.15995;
    tform[13] = 0.0;
    tform[14] = 0.9465;
    d_joint.setFixedTransform(tform);
    d_body.BodyInternal->JointInternal = d_joint.copy(d_lobj_4[1]);
    b_robot->d_addBody(d_body, gobj_5[6], gobj_6[6], gobj_7[3]);
    d_body.matlabCodegenDestructor();
    d_lobj_2.matlabCodegenDestructor();
    d_lobj_2.Base.matlabCodegenDestructor();
    d_lobj_2._pobj0.matlabCodegenDestructor();
    d_lobj_5.matlabCodegenDestructor();
    d_lobj_2.Base.CollisionsInternal.matlabCodegenDestructor();
    d_lobj_2._pobj0.CollisionsInternal.matlabCodegenDestructor();
    d_lobj_3.matlabCodegenDestructor();
    d_lobj_5._pobj0.matlabCodegenDestructor();
    //  Arm joints 1-6
    e_lobj_5._pobj0.matlabCodegenIsDeleted = true;
    e_lobj_3.matlabCodegenIsDeleted = true;
    e_lobj_2._pobj0.CollisionsInternal.matlabCodegenIsDeleted = true;
    e_lobj_2.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
    e_lobj_5.matlabCodegenIsDeleted = true;
    e_lobj_2._pobj0.matlabCodegenIsDeleted = true;
    e_lobj_2.Base.matlabCodegenIsDeleted = true;
    e_lobj_2.matlabCodegenIsDeleted = true;
    e_body.matlabCodegenIsDeleted = true;
    e_body.e_init(e_lobj_2, e_lobj_3, e_lobj_4[0], e_lobj_5);
    e_joint.f_init();
    for (int i{0}; i < 16; i++) {
      tform[i] = iv[i];
    }
    tform[12] = -0.0011149;
    tform[13] = 0.0446;
    tform[14] = 0.0;
    e_joint.setFixedTransform(tform);
    e_joint.b_set_JointAxis();
    e_joint.c_set_PositionLimits();
    e_body.BodyInternal->JointInternal = e_joint.copy(e_lobj_4[1]);
    b_robot->e_addBody(e_body, gobj_5[8], gobj_6[8], gobj_7[4]);
    e_body.matlabCodegenDestructor();
    e_lobj_2.matlabCodegenDestructor();
    e_lobj_2.Base.matlabCodegenDestructor();
    e_lobj_2._pobj0.matlabCodegenDestructor();
    e_lobj_5.matlabCodegenDestructor();
    e_lobj_2.Base.CollisionsInternal.matlabCodegenDestructor();
    e_lobj_2._pobj0.CollisionsInternal.matlabCodegenDestructor();
    e_lobj_3.matlabCodegenDestructor();
    e_lobj_5._pobj0.matlabCodegenDestructor();
    f_lobj_5._pobj0.matlabCodegenIsDeleted = true;
    f_lobj_3.matlabCodegenIsDeleted = true;
    f_lobj_2._pobj0.CollisionsInternal.matlabCodegenIsDeleted = true;
    f_lobj_2.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
    f_lobj_5.matlabCodegenIsDeleted = true;
    f_lobj_2._pobj0.matlabCodegenIsDeleted = true;
    f_lobj_2.Base.matlabCodegenIsDeleted = true;
    f_lobj_2.matlabCodegenIsDeleted = true;
    f_body.matlabCodegenIsDeleted = true;
    f_body.f_init(f_lobj_2, f_lobj_3, f_lobj_4[0], f_lobj_5);
    f_joint.g_init();
    for (int i{0}; i < 16; i++) {
      tform[i] = iv[i];
    }
    tform[12] = 0.0;
    tform[13] = 0.1061;
    tform[14] = 0.0;
    f_joint.setFixedTransform(tform);
    f_joint.d_set_JointAxis();
    f_joint.d_set_PositionLimits();
    f_body.BodyInternal->JointInternal = f_joint.copy(f_lobj_4[1]);
    b_robot->f_addBody(f_body, gobj_5[10], gobj_6[10], gobj_7[5]);
    f_body.matlabCodegenDestructor();
    f_lobj_2.matlabCodegenDestructor();
    f_lobj_2.Base.matlabCodegenDestructor();
    f_lobj_2._pobj0.matlabCodegenDestructor();
    f_lobj_5.matlabCodegenDestructor();
    f_lobj_2.Base.CollisionsInternal.matlabCodegenDestructor();
    f_lobj_2._pobj0.CollisionsInternal.matlabCodegenDestructor();
    f_lobj_3.matlabCodegenDestructor();
    f_lobj_5._pobj0.matlabCodegenDestructor();
    addBodyWithJoint(b_robot, gobj_5[12], gobj_6[12], gobj_7[6]);
    b_addBodyWithJoint(b_robot, gobj_5[14], gobj_6[14], gobj_7[7]);
    c_addBodyWithJoint(b_robot, gobj_5[16], gobj_6[16], gobj_7[8]);
    d_addBodyWithJoint(b_robot, gobj_5[18], gobj_6[18], gobj_7[9]);
    //  Fixed gripper link
    g_lobj_5._pobj0.matlabCodegenIsDeleted = true;
    g_lobj_3.matlabCodegenIsDeleted = true;
    g_lobj_2._pobj0.CollisionsInternal.matlabCodegenIsDeleted = true;
    g_lobj_2.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
    g_lobj_5.matlabCodegenIsDeleted = true;
    g_lobj_2._pobj0.matlabCodegenIsDeleted = true;
    g_lobj_2.Base.matlabCodegenIsDeleted = true;
    g_lobj_2.matlabCodegenIsDeleted = true;
    g_body.matlabCodegenIsDeleted = true;
    g_body.k_init(g_lobj_2, g_lobj_3, g_lobj_4[0], g_lobj_5);
    g_joint.l_init();
    for (int i{0}; i < 16; i++) {
      tform[i] = iv[i];
    }
    tform[12] = 0.1039;
    tform[13] = 0.0;
    tform[14] = 0.0;
    g_joint.setFixedTransform(tform);
    g_body.BodyInternal->JointInternal = g_joint.copy(g_lobj_4[1]);
    b_robot->k_addBody(g_body, gobj_5[20], gobj_6[20], gobj_7[10]);
    g_body.matlabCodegenDestructor();
    g_lobj_2.matlabCodegenDestructor();
    g_lobj_2.Base.matlabCodegenDestructor();
    g_lobj_2._pobj0.matlabCodegenDestructor();
    g_lobj_5.matlabCodegenDestructor();
    g_lobj_2.Base.CollisionsInternal.matlabCodegenDestructor();
    g_lobj_2._pobj0.CollisionsInternal.matlabCodegenDestructor();
    g_lobj_3.matlabCodegenDestructor();
    g_lobj_5._pobj0.matlabCodegenDestructor();
    robotPersist = b_robot;
    robotPersist_not_empty = true;
  }
  return robotPersist;
}

} // namespace codegen
} // namespace gik9dof
void loadRobotForCodegen_delete()
{
  coder::robotics::manip::internal::RigidBody *obj;
  for (int i{0}; i < 11; i++) {
    obj = &gobj_7[i];
    if (!obj->matlabCodegenIsDeleted) {
      obj->matlabCodegenIsDeleted = true;
    }
  }
  for (int i{0}; i < 22; i++) {
    gobj_5[i].matlabCodegenDestructor();
  }
  for (int i{0}; i < 11; i++) {
    gobj_7[i]._pobj0.matlabCodegenDestructor();
  }
}

void loadRobotForCodegen_init()
{
  robotPersist_not_empty = false;
}

void loadRobotForCodegen_new()
{
  for (int i{0}; i < 11; i++) {
    gobj_7[i]._pobj0.matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 22; i++) {
    gobj_5[i].matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 11; i++) {
    gobj_7[i].matlabCodegenIsDeleted = true;
  }
}

// End of code generation (loadRobotForCodegen.cpp)
