//
// solveGIKStep.cpp
//
// Code generation for function 'solveGIKStep'
//

// Include files
#include "solveGIKStep.h"
#include "CollisionSet.h"
#include "DistanceBoundsConstraint.h"
#include "ErrorDampedLevenbergMarquardt.h"
#include "GIKProblem.h"
#include "JointPositionBounds.h"
#include "PoseTarget.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "generalizedInverseKinematics.h"
#include "gik9dof_codegen_followTrajectory_data.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"

// Function Definitions
void solveGIKStep_delete()
{
  coder::robotics::manip::internal::RigidBody *obj;
  if (!robot.matlabCodegenIsDeleted) {
    robot.matlabCodegenIsDeleted = true;
  }
  solver.matlabCodegenDestructor();
  if (!solver._pobj4.matlabCodegenIsDeleted) {
    solver._pobj4.matlabCodegenIsDeleted = true;
  }
  if (!solver.Problem.matlabCodegenIsDeleted) {
    solver.Problem.matlabCodegenIsDeleted = true;
  }
  if (!solver.Problem._pobj0.matlabCodegenIsDeleted) {
    solver.Problem._pobj0.matlabCodegenIsDeleted = true;
  }
  if (!solver.Problem._pobj1.matlabCodegenIsDeleted) {
    solver.Problem._pobj1.matlabCodegenIsDeleted = true;
  }
  if (!solver.Problem._pobj2.matlabCodegenIsDeleted) {
    solver.Problem._pobj2.matlabCodegenIsDeleted = true;
  }
  if (!solver._pobj3.matlabCodegenIsDeleted) {
    solver._pobj3.matlabCodegenIsDeleted = true;
  }
  if (!robot.TreeInternal.matlabCodegenIsDeleted) {
    robot.TreeInternal.matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 12; i++) {
    obj = &solver._pobj1[i];
    if (!obj->matlabCodegenIsDeleted) {
      obj->matlabCodegenIsDeleted = true;
    }
  }
  if (!solver._pobj3.Base.matlabCodegenIsDeleted) {
    solver._pobj3.Base.matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 12; i++) {
    obj = &solver._pobj3._pobj2[i];
    if (!obj->matlabCodegenIsDeleted) {
      obj->matlabCodegenIsDeleted = true;
    }
  }
  if (!robot.TreeInternal.Base.matlabCodegenIsDeleted) {
    robot.TreeInternal.Base.matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 12; i++) {
    obj = &robot.TreeInternal._pobj2[i];
    if (!obj->matlabCodegenIsDeleted) {
      obj->matlabCodegenIsDeleted = true;
    }
  }
  for (int i{0}; i < 12; i++) {
    solver._pobj1[i]._pobj0.matlabCodegenDestructor();
  }
  for (int i{0}; i < 25; i++) {
    solver._pobj2[i].matlabCodegenDestructor();
  }
  solver._pobj3.Base._pobj0.matlabCodegenDestructor();
  solver._pobj3._pobj0[0].matlabCodegenDestructor();
  solver._pobj3._pobj0[1].matlabCodegenDestructor();
  solver._pobj3._pobj0[2].matlabCodegenDestructor();
  for (int i{0}; i < 12; i++) {
    solver._pobj3._pobj2[i]._pobj0.matlabCodegenDestructor();
  }
  robot.TreeInternal.Base._pobj0.matlabCodegenDestructor();
  robot.TreeInternal._pobj0[0].matlabCodegenDestructor();
  robot.TreeInternal._pobj0[1].matlabCodegenDestructor();
  robot.TreeInternal._pobj0[2].matlabCodegenDestructor();
  for (int i{0}; i < 12; i++) {
    robot.TreeInternal._pobj2[i]._pobj0.matlabCodegenDestructor();
  }
  robot._pobj0.matlabCodegenDestructor();
}

void solveGIKStep_init()
{
  solver_not_empty = false;
}

void solveGIKStep_new()
{
  robot._pobj0.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 12; i++) {
    robot.TreeInternal._pobj2[i]._pobj0.matlabCodegenIsDeleted = true;
  }
  robot.TreeInternal._pobj0[0].matlabCodegenIsDeleted = true;
  robot.TreeInternal._pobj0[1].matlabCodegenIsDeleted = true;
  robot.TreeInternal._pobj0[2].matlabCodegenIsDeleted = true;
  robot.TreeInternal.Base._pobj0.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 12; i++) {
    solver._pobj3._pobj2[i]._pobj0.matlabCodegenIsDeleted = true;
  }
  solver._pobj3._pobj0[0].matlabCodegenIsDeleted = true;
  solver._pobj3._pobj0[1].matlabCodegenIsDeleted = true;
  solver._pobj3._pobj0[2].matlabCodegenIsDeleted = true;
  solver._pobj3.Base._pobj0.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 25; i++) {
    solver._pobj2[i].matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 12; i++) {
    solver._pobj1[i]._pobj0.matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 12; i++) {
    robot.TreeInternal._pobj2[i].matlabCodegenIsDeleted = true;
  }
  robot.TreeInternal.Base.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 12; i++) {
    solver._pobj3._pobj2[i].matlabCodegenIsDeleted = true;
  }
  solver._pobj3.Base.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 12; i++) {
    solver._pobj1[i].matlabCodegenIsDeleted = true;
  }
  robot.TreeInternal.matlabCodegenIsDeleted = true;
  solver._pobj3.matlabCodegenIsDeleted = true;
  solver.Problem._pobj2.matlabCodegenIsDeleted = true;
  solver.Problem._pobj1.matlabCodegenIsDeleted = true;
  solver.Problem._pobj0.matlabCodegenIsDeleted = true;
  solver.Problem.matlabCodegenIsDeleted = true;
  solver._pobj4.matlabCodegenIsDeleted = true;
  solver.matlabCodegenIsDeleted = true;
  robot.matlabCodegenIsDeleted = true;
}

// End of code generation (solveGIKStep.cpp)
