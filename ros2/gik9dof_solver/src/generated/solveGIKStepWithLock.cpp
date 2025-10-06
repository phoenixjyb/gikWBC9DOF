//
// solveGIKStepWithLock.cpp
//
// Code generation for function 'solveGIKStepWithLock'
//

// Include files
#include "solveGIKStepWithLock.h"
#include "CollisionSet.h"
#include "DistanceBoundsConstraint.h"
#include "ErrorDampedLevenbergMarquardt.h"
#include "GIKProblem.h"
#include "JointPositionBounds.h"
#include "PoseTarget.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "constraintDistanceBounds.h"
#include "constraintJointBounds.h"
#include "constraintPoseTarget.h"
#include "generalizedInverseKinematics.h"
#include "gik9dof_codegen_stagedFollowTrajectory_data.h"
#include "loadRobotForCodegen.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <algorithm>

// Function Definitions
namespace gik9dof {
namespace codegen {
void solveGIKStepWithLock(const double qCurrent[9], const double targetPose[16],
                          double distanceLower, double distanceWeight,
                          const bool lockMask[9], double qNext[9])
{
  int loop_ub;
  // SOLVEGIKSTEPWITHLOCK Generalized IK step with optional joint locks.
  if (!solver_not_empty) {
    int b_iv[2];
    b_robot = loadRobotForCodegen();
    solver.init(b_robot);
    solver_not_empty = true;
    poseConstraint.init();
    jointConstraint.init(b_robot);
    distanceConstraint.init();
    distanceConstraint.ReferenceBody.reserve(200);
    b_robot->get_BaseName((char *)distanceConstraint.ReferenceBody.data(),
                          b_iv);
    (*(int(*)[2])distanceConstraint.ReferenceBody.size())[0] = b_iv[0];
    (*(int(*)[2])distanceConstraint.ReferenceBody.size())[1] = b_iv[1];
    distanceConstraint.ReferenceBody.set_size(
        distanceConstraint.ReferenceBody.size(0),
        distanceConstraint.ReferenceBody.size(1));
    jointBoundsDefault.set_size(jointConstraint.BoundsInternal.size(0), 2);
    loop_ub = jointConstraint.BoundsInternal.size(0) << 1;
    for (int i{0}; i < loop_ub; i++) {
      jointBoundsDefault[i] = jointConstraint.BoundsInternal[i];
    }
  }
  std::copy(&targetPose[0], &targetPose[16],
            &poseConstraint.TargetTransform[0]);
  if (distanceLower <= 0.0) {
    distanceConstraint.Bounds[0] = 0.0;
  } else {
    distanceConstraint.Bounds[0] = distanceLower;
  }
  distanceConstraint.Bounds[1] = rtInf;
  if (distanceWeight < 0.0) {
    distanceConstraint.Weights = 0.0;
  } else {
    distanceConstraint.Weights = distanceWeight;
  }
  //  Restore default joint bounds then apply lock mask.
  jointConstraint.BoundsInternal.set_size(jointBoundsDefault.size(0), 2);
  loop_ub = jointBoundsDefault.size(0) << 1;
  for (int i{0}; i < loop_ub; i++) {
    jointConstraint.BoundsInternal[i] = jointBoundsDefault[i];
  }
  for (loop_ub = 0; loop_ub < 9; loop_ub++) {
    if (lockMask[loop_ub]) {
      double d;
      d = qCurrent[loop_ub];
      jointConstraint.BoundsInternal[loop_ub] = d;
      jointConstraint
          .BoundsInternal[loop_ub + jointConstraint.BoundsInternal.size(0)] = d;
    }
  }
  solver.step(qCurrent, poseConstraint, jointConstraint, distanceConstraint,
              qNext);
}

} // namespace codegen
} // namespace gik9dof
void solveGIKStepWithLock_delete()
{
  coder::robotics::manip::internal::RigidBody *obj;
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
}

void solveGIKStepWithLock_init()
{
  solver_not_empty = false;
}

void solveGIKStepWithLock_new()
{
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
    solver._pobj3._pobj2[i].matlabCodegenIsDeleted = true;
  }
  solver._pobj3.Base.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 12; i++) {
    solver._pobj1[i].matlabCodegenIsDeleted = true;
  }
  solver._pobj3.matlabCodegenIsDeleted = true;
  solver.Problem._pobj2.matlabCodegenIsDeleted = true;
  solver.Problem._pobj1.matlabCodegenIsDeleted = true;
  solver.Problem._pobj0.matlabCodegenIsDeleted = true;
  solver.Problem.matlabCodegenIsDeleted = true;
  solver._pobj4.matlabCodegenIsDeleted = true;
  solver.matlabCodegenIsDeleted = true;
}

// End of code generation (solveGIKStepWithLock.cpp)
