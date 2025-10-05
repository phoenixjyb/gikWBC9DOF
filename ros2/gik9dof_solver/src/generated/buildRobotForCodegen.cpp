//
// buildRobotForCodegen.cpp
//
// Code generation for function 'buildRobotForCodegen'
//

// Include files
#include "buildRobotForCodegen.h"
#include "CollisionSet.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "gik9dof_codegen_followTrajectory_data.h"
#include "rigidBody1.h"
#include "rigidBodyJoint.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace gik9dof {
namespace codegen {
void addBodyWithJoint(coder::rigidBodyTree *tree,
                      coder::robotics::manip::internal::CollisionSet &iobj_0,
                      coder::rigidBodyJoint &iobj_1,
                      coder::robotics::manip::internal::RigidBody &iobj_2)
{
  coder::rigidBody body;
  coder::rigidBodyJoint lobj_4[2];
  coder::rigidBodyJoint joint;
  coder::robotics::manip::internal::CollisionSet lobj_3;
  coder::robotics::manip::internal::RigidBody lobj_5;
  coder::robotics::manip::internal::RigidBodyTree lobj_2;
  double tform[16];
  lobj_5._pobj0.matlabCodegenIsDeleted = true;
  lobj_3.matlabCodegenIsDeleted = true;
  lobj_2._pobj0.CollisionsInternal.matlabCodegenIsDeleted = true;
  lobj_2.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
  lobj_5.matlabCodegenIsDeleted = true;
  lobj_2._pobj0.matlabCodegenIsDeleted = true;
  lobj_2.Base.matlabCodegenIsDeleted = true;
  lobj_2.matlabCodegenIsDeleted = true;
  body.matlabCodegenIsDeleted = true;
  body.g_init(lobj_2, lobj_3, lobj_4[0], lobj_5);
  joint.h_init();
  for (int i{0}; i < 16; i++) {
    tform[i] = iv[i];
  }
  tform[12] = -0.34928;
  tform[13] = 0.019998;
  tform[14] = 0.0;
  joint.setFixedTransform(tform);
  joint.d_set_JointAxis();
  joint.e_set_PositionLimits();
  body.BodyInternal->JointInternal = joint.copy(lobj_4[1]);
  tree->g_addBody(body, (&iobj_0)[0], (&iobj_1)[0], iobj_2);
  lobj_2.Base.matlabCodegenDestructor();
  lobj_2._pobj0.matlabCodegenDestructor();
  lobj_2.Base.CollisionsInternal.matlabCodegenDestructor();
  lobj_2._pobj0.CollisionsInternal.matlabCodegenDestructor();
  lobj_5._pobj0.matlabCodegenDestructor();
}

void b_addBodyWithJoint(coder::rigidBodyTree *tree,
                        coder::robotics::manip::internal::CollisionSet &iobj_0,
                        coder::rigidBodyJoint &iobj_1,
                        coder::robotics::manip::internal::RigidBody &iobj_2)
{
  coder::rigidBody body;
  coder::rigidBodyJoint lobj_4[2];
  coder::rigidBodyJoint joint;
  coder::robotics::manip::internal::CollisionSet lobj_3;
  coder::robotics::manip::internal::RigidBody lobj_5;
  coder::robotics::manip::internal::RigidBodyTree lobj_2;
  double tform[16];
  lobj_5._pobj0.matlabCodegenIsDeleted = true;
  lobj_3.matlabCodegenIsDeleted = true;
  lobj_2._pobj0.CollisionsInternal.matlabCodegenIsDeleted = true;
  lobj_2.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
  lobj_5.matlabCodegenIsDeleted = true;
  lobj_2._pobj0.matlabCodegenIsDeleted = true;
  lobj_2.Base.matlabCodegenIsDeleted = true;
  lobj_2.matlabCodegenIsDeleted = true;
  body.matlabCodegenIsDeleted = true;
  body.h_init(lobj_2, lobj_3, lobj_4[0], lobj_5);
  joint.i_init();
  for (int i{0}; i < 16; i++) {
    tform[i] = iv[i];
  }
  tform[12] = 0.02735;
  tform[13] = 0.069767;
  tform[14] = 0.0;
  joint.setFixedTransform(tform);
  joint.set_JointAxis();
  joint.c_set_PositionLimits();
  body.BodyInternal->JointInternal = joint.copy(lobj_4[1]);
  tree->h_addBody(body, (&iobj_0)[0], (&iobj_1)[0], iobj_2);
  lobj_2.Base.matlabCodegenDestructor();
  lobj_2._pobj0.matlabCodegenDestructor();
  lobj_2.Base.CollisionsInternal.matlabCodegenDestructor();
  lobj_2._pobj0.CollisionsInternal.matlabCodegenDestructor();
  lobj_5._pobj0.matlabCodegenDestructor();
}

void c_addBodyWithJoint(coder::rigidBodyTree *tree,
                        coder::robotics::manip::internal::CollisionSet &iobj_0,
                        coder::rigidBodyJoint &iobj_1,
                        coder::robotics::manip::internal::RigidBody &iobj_2)
{
  coder::rigidBody body;
  coder::rigidBodyJoint lobj_4[2];
  coder::rigidBodyJoint joint;
  coder::robotics::manip::internal::CollisionSet lobj_3;
  coder::robotics::manip::internal::RigidBody lobj_5;
  coder::robotics::manip::internal::RigidBodyTree lobj_2;
  double tform[16];
  lobj_5._pobj0.matlabCodegenIsDeleted = true;
  lobj_3.matlabCodegenIsDeleted = true;
  lobj_2._pobj0.CollisionsInternal.matlabCodegenIsDeleted = true;
  lobj_2.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
  lobj_5.matlabCodegenIsDeleted = true;
  lobj_2._pobj0.matlabCodegenIsDeleted = true;
  lobj_2.Base.matlabCodegenIsDeleted = true;
  lobj_2.matlabCodegenIsDeleted = true;
  body.matlabCodegenIsDeleted = true;
  body.i_init(lobj_2, lobj_3, lobj_4[0], lobj_5);
  joint.j_init();
  for (int i{0}; i < 16; i++) {
    tform[i] = iv[i];
  }
  tform[12] = 0.2463;
  tform[13] = -0.00049894;
  tform[14] = 0.0;
  joint.setFixedTransform(tform);
  joint.b_set_JointAxis();
  joint.f_set_PositionLimits();
  body.BodyInternal->JointInternal = joint.copy(lobj_4[1]);
  tree->i_addBody(body, (&iobj_0)[0], (&iobj_1)[0], iobj_2);
  lobj_2.Base.matlabCodegenDestructor();
  lobj_2._pobj0.matlabCodegenDestructor();
  lobj_2.Base.CollisionsInternal.matlabCodegenDestructor();
  lobj_2._pobj0.CollisionsInternal.matlabCodegenDestructor();
  lobj_5._pobj0.matlabCodegenDestructor();
}

void d_addBodyWithJoint(coder::rigidBodyTree *tree,
                        coder::robotics::manip::internal::CollisionSet &iobj_0,
                        coder::rigidBodyJoint &iobj_1,
                        coder::robotics::manip::internal::RigidBody &iobj_2)
{
  coder::rigidBody body;
  coder::rigidBodyJoint lobj_4[2];
  coder::rigidBodyJoint joint;
  coder::robotics::manip::internal::CollisionSet lobj_3;
  coder::robotics::manip::internal::RigidBody lobj_5;
  coder::robotics::manip::internal::RigidBodyTree lobj_2;
  double tform[16];
  lobj_5._pobj0.matlabCodegenIsDeleted = true;
  lobj_3.matlabCodegenIsDeleted = true;
  lobj_2._pobj0.CollisionsInternal.matlabCodegenIsDeleted = true;
  lobj_2.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
  lobj_5.matlabCodegenIsDeleted = true;
  lobj_2._pobj0.matlabCodegenIsDeleted = true;
  lobj_2.Base.matlabCodegenIsDeleted = true;
  lobj_2.matlabCodegenIsDeleted = true;
  body.matlabCodegenIsDeleted = true;
  body.j_init(lobj_2, lobj_3, lobj_4[0], lobj_5);
  joint.k_init();
  for (int i{0}; i < 16; i++) {
    tform[i] = iv[i];
  }
  tform[12] = 0.058249;
  tform[13] = 0.00050025;
  tform[14] = 0.0;
  joint.setFixedTransform(tform);
  joint.set_JointAxis();
  joint.c_set_PositionLimits();
  body.BodyInternal->JointInternal = joint.copy(lobj_4[1]);
  tree->j_addBody(body, (&iobj_0)[0], (&iobj_1)[0], iobj_2);
  lobj_2.Base.matlabCodegenDestructor();
  lobj_2._pobj0.matlabCodegenDestructor();
  lobj_2.Base.CollisionsInternal.matlabCodegenDestructor();
  lobj_2._pobj0.CollisionsInternal.matlabCodegenDestructor();
  lobj_5._pobj0.matlabCodegenDestructor();
}

} // namespace codegen
} // namespace gik9dof

// End of code generation (buildRobotForCodegen.cpp)
