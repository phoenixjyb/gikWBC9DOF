//
// File: solveGIKStepRealtime.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

// Include Files
#include "solveGIKStepRealtime.h"
#include "GIKSolver.h"
#include "constraintDistanceBounds.h"
#include "constraintJointBounds.h"
#include "constraintPoseTarget.h"
#include "generalizedInverseKinematics.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <algorithm>
#include <cstring>

// Function Definitions
//
// SOLVEGIKSTEPREALTIME Single GIK step for real-time deployment (no persistent
// variables)
//    This version removes persistent variables and accepts pre-constructed
//    solver and robot objects for code generation compatibility.
//
//    Inputs:
//        robot         - rigidBodyTree object (9 DOF)
//        solver        - generalizedInverseKinematics object
//        qCurrent      - Current joint configuration (9x1 double)
//        targetPose    - Target end-effector pose (4x4 homogeneous transform)
//        distanceLower - Lower bound for distance constraint (scalar)
//        distanceWeight - Weight for distance constraint (scalar)
//
//    Outputs:
//        qNext      - Next joint configuration (9x1 double)
//        solverInfo - Solver diagnostics structure
//
//
// Arguments    : GIKSolver *aInstancePtr
//                coder::rigidBodyTree &robot
//                coder::generalizedInverseKinematics &solver
//                const double qCurrent[9]
//                const double targetPose[16]
//                double distanceLower
//                double distanceWeight
//                double qNext[9]
//                struct1_T solverInfo_ConstraintViolations[3]
//                char solverInfo_Status_data[]
//                int solverInfo_Status_size[2]
//                double &solverInfo_NumRandomRestarts
//                double &solverInfo_ExitFlag
// Return Type  : double
//
namespace gik9dof {
namespace codegen_inuse {
double solveGIKStepRealtime(
    GIKSolver *aInstancePtr, coder::rigidBodyTree &robot,
    coder::generalizedInverseKinematics &solver, const double qCurrent[9],
    const double targetPose[16], double distanceLower, double distanceWeight,
    double qNext[9], struct1_T solverInfo_ConstraintViolations[3],
    char solverInfo_Status_data[], int solverInfo_Status_size[2],
    double &solverInfo_NumRandomRestarts, double &solverInfo_ExitFlag)
{
  coder::constraintDistanceBounds distanceConstraint;
  coder::constraintJointBounds jointConstraint;
  coder::constraintPoseTarget poseConstraint;
  int b_iv[2];
  //  Validate inputs
  //  Force function call for better debugging
  //  Create constraints (these are lightweight objects)
  poseConstraint.init();
  std::copy(&targetPose[0], &targetPose[16],
            &poseConstraint.TargetTransform[0]);
  jointConstraint.init(robot);
  distanceConstraint.init();
  distanceConstraint.ReferenceBody.reserve(200);
  robot.get_BaseName((char *)distanceConstraint.ReferenceBody.data(), b_iv);
  (*(int(*)[2])distanceConstraint.ReferenceBody.size())[0] = b_iv[0];
  (*(int(*)[2])distanceConstraint.ReferenceBody.size())[1] = b_iv[1];
  distanceConstraint.ReferenceBody.set_size(
      distanceConstraint.ReferenceBody.size(0),
      distanceConstraint.ReferenceBody.size(1));
  //  Process distance bounds
  //  Use large finite upper bound instead of inf (MATLAB constraint system
  //  requires finite bounds) 100 meters is effectively unlimited for a mobile
  //  manipulator
  if (distanceLower <= 0.0) {
    distanceConstraint.Bounds[0] = 0.0;
  } else {
    distanceConstraint.Bounds[0] = distanceLower;
  }
  distanceConstraint.Bounds[1] = 100.0;
  if (distanceWeight < 0.0) {
    distanceConstraint.Weights = 0.0;
  } else {
    distanceConstraint.Weights = distanceWeight;
  }
  //  Solve IK
  return solver.step(aInstancePtr, qCurrent, poseConstraint, jointConstraint,
                     distanceConstraint, qNext, solverInfo_ConstraintViolations,
                     solverInfo_Status_data, solverInfo_Status_size,
                     solverInfo_NumRandomRestarts, solverInfo_ExitFlag);
}

} // namespace codegen_inuse
} // namespace gik9dof

//
// File trailer for solveGIKStepRealtime.cpp
//
// [EOF]
//
