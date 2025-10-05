//
// gik9dof_codegen_followTrajectory.cpp
//
// Code generation for function 'gik9dof_codegen_followTrajectory'
//

// Include files
#include "gik9dof_codegen_followTrajectory.h"
#include "constraintDistanceBounds.h"
#include "constraintJointBounds.h"
#include "constraintPoseTarget.h"
#include "generalizedInverseKinematics.h"
#include "gik9dof_codegen_followTrajectory_data.h"
#include "gik9dof_codegen_followTrajectory_initialize.h"
#include "loadRobotForCodegen.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <algorithm>

// Function Definitions
namespace gik9dof {
namespace codegen {
void followTrajectory(const double q0[9], const double poses_data[],
                      const int poses_size[3], double distanceLower,
                      double distanceWeight, double qOut[9])
{
  static coder::constraintDistanceBounds distanceConstraint;
  static coder::constraintJointBounds jointConstraint;
  static coder::constraintPoseTarget poseConstraint;
  double b_qOut[9];
  int i;
  if (!isInitialized_gik9dof_codegen_followTrajectory) {
    gik9dof_codegen_followTrajectory_initialize();
  }
  // FOLLOWTRAJECTORY Iterate solveGIKStep across a pose sequence.
  std::copy(&q0[0], &q0[9], &qOut[0]);
  i = poses_size[2];
  for (int k{0}; k < i; k++) {
    // SOLVEGIKSTEP Single generalized IK step for code generation.
    if (!solver_not_empty) {
      int b_iv[2];
      loadRobotForCodegen();
      solver.init(robot);
      solver_not_empty = true;
      poseConstraint.init();
      jointConstraint.init(robot);
      distanceConstraint.init();
      distanceConstraint.ReferenceBody.reserve(200);
      robot.get_BaseName((char *)distanceConstraint.ReferenceBody.data(), b_iv);
      (*(int(*)[2])distanceConstraint.ReferenceBody.size())[0] = b_iv[0];
      (*(int(*)[2])distanceConstraint.ReferenceBody.size())[1] = b_iv[1];
      distanceConstraint.ReferenceBody.set_size(
          distanceConstraint.ReferenceBody.size(0),
          distanceConstraint.ReferenceBody.size(1));
    }
    for (int i1{0}; i1 < 4; i1++) {
      int i2;
      int i3;
      i2 = i1 << 2;
      i3 = 4 * i1 + 16 * k;
      poseConstraint.TargetTransform[i2] = poses_data[i3];
      poseConstraint.TargetTransform[i2 + 1] = poses_data[i3 + 1];
      poseConstraint.TargetTransform[i2 + 2] = poses_data[i3 + 2];
      poseConstraint.TargetTransform[i2 + 3] = poses_data[i3 + 3];
    }
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
    std::copy(&qOut[0], &qOut[9], &b_qOut[0]);
    solver.step(b_qOut, poseConstraint, jointConstraint, distanceConstraint,
                qOut);
  }
}

} // namespace codegen
} // namespace gik9dof

// End of code generation (gik9dof_codegen_followTrajectory.cpp)
