//
// File: gik9dof_codegen_inuse_solveGIKStepWrapper.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

// Include Files
#include "gik9dof_codegen_inuse_solveGIKStepWrapper.h"
#include "CollisionSet.h"
#include "DistanceBoundsConstraint.h"
#include "ErrorDampedLevenbergMarquardt.h"
#include "GIKProblem.h"
#include "JointPositionBounds.h"
#include "PoseTarget.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "buildRobotForCodegen.h"
#include "constraintDistanceBounds.h"
#include "constraintJointBounds.h"
#include "constraintPoseTarget.h"
#include "generalizedInverseKinematics.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_data.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_initialize.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "rigidBodyJoint.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <algorithm>
#include <cstring>

// Variable Definitions
static coder::robotics::manip::internal::RigidBody gobj_7[11];

static coder::robotics::manip::internal::CollisionSet gobj_5[22];

static boolean_T bodyNames_not_empty;

static coder::generalizedInverseKinematics solver;

static boolean_T solver_not_empty;

static coder::rigidBodyTree robot;

// Function Definitions
//
// SOLVEGIKSTEPWRAPPER GIK solver with 20 distance constraints for code
// generation
//    This version uses persistent variables pattern (proven to work with MATLAB
//    Coder)
//
//
// Arguments    : const double qCurrent[9]
//                const double targetPose[16]
//                const int distBodyIndices[20]
//                const int distRefBodyIndices[20]
//                const double distBoundsLower[20]
//                const double distBoundsUpper[20]
//                const double distWeights[20]
//                double qNext[9]
//                struct0_T *solverInfo
// Return Type  : void
//
namespace gik9dof {
namespace codegen_inuse {
void solveGIKStepWrapper(const double qCurrent[9], const double targetPose[16],
                         const int distBodyIndices[20],
                         const int distRefBodyIndices[20],
                         const double distBoundsLower[20],
                         const double distBoundsUpper[20],
                         const double distWeights[20], double qNext[9],
                         struct0_T *solverInfo)
{
  static coder::constraintDistanceBounds gobj_4[20];
  static coder::constraintDistanceBounds *distConstraints[20];
  static coder::constraintJointBounds jointConstraint;
  static coder::constraintPoseTarget poseConstraint;
  static coder::rigidBodyJoint gobj_6[22];
  double lowerBound;
  double params_DampingBias;
  double params_ErrorChangeTolerance;
  double params_SolutionTolerance;
  double params_StepTolerance;
  double upperBound;
  double weight;
  boolean_T b_expl_temp;
  boolean_T params_RandomRestart;
  boolean_T params_UseErrorDamping;
  if (!isInitialized_gik9dof_codegen_inuse_solveGIKStepWrapper) {
    gik9dof_codegen_inuse_solveGIKStepWrapper_initialize();
  }
  //  Fixed body name list
  bodyNames_not_empty = true;
  //  Persistent solver and constraints
  if (!solver_not_empty) {
    char expl_temp[18];
    //  Build robot model procedurally
    buildRobotForCodegen(gobj_5[0], gobj_6[0], gobj_7[0], robot);
    //  Create GIK solver with 22 constraint inputs (1 pose + 1 joint + 20
    //  distance)
    solver.init(robot);
    solver_not_empty = true;
    //  Configure solver parameters for real-time performance
    weight = solver.Solver->getSolverParams(
        expl_temp, lowerBound, upperBound, params_SolutionTolerance,
        b_expl_temp, params_RandomRestart, params_StepTolerance,
        params_ErrorChangeTolerance, params_DampingBias,
        params_UseErrorDamping);
    b_expl_temp = solver.EnforceJointLimits;
    solver.set_SolverParameters(
        weight, 0.05, upperBound, params_SolutionTolerance, b_expl_temp,
        params_RandomRestart, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    solver.Solver->getSolverParams(expl_temp, lowerBound, upperBound,
                                   params_SolutionTolerance, b_expl_temp,
                                   params_RandomRestart, params_StepTolerance,
                                   params_ErrorChangeTolerance,
                                   params_DampingBias, params_UseErrorDamping);
    b_expl_temp = solver.EnforceJointLimits;
    solver.set_SolverParameters(
        50.0, lowerBound, upperBound, params_SolutionTolerance, b_expl_temp,
        params_RandomRestart, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    weight = solver.Solver->getSolverParams(
        expl_temp, lowerBound, upperBound, params_SolutionTolerance,
        b_expl_temp, params_RandomRestart, params_StepTolerance,
        params_ErrorChangeTolerance, params_DampingBias,
        params_UseErrorDamping);
    b_expl_temp = solver.EnforceJointLimits;
    solver.set_SolverParameters(
        weight, lowerBound, upperBound, params_SolutionTolerance, b_expl_temp,
        false, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    weight = solver.Solver->getSolverParams(
        expl_temp, lowerBound, upperBound, params_SolutionTolerance,
        b_expl_temp, params_RandomRestart, params_StepTolerance,
        params_ErrorChangeTolerance, params_DampingBias,
        params_UseErrorDamping);
    b_expl_temp = solver.EnforceJointLimits;
    solver.set_SolverParameters(
        weight, lowerBound, upperBound, 1.0E-6, b_expl_temp,
        params_RandomRestart, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    weight = solver.Solver->getSolverParams(
        expl_temp, lowerBound, upperBound, params_SolutionTolerance,
        b_expl_temp, params_RandomRestart, params_StepTolerance,
        params_ErrorChangeTolerance, params_DampingBias,
        params_UseErrorDamping);
    b_expl_temp = solver.EnforceJointLimits;
    solver.set_SolverParameters(
        weight, lowerBound, 1.0E-7, params_SolutionTolerance, b_expl_temp,
        params_RandomRestart, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    //  Create pose and joint constraints
    poseConstraint.init();
    jointConstraint.init(robot);
    //  Create 20 distance constraints (all initialized)
    distConstraints[0] = gobj_4[0].init();
    distConstraints[0]->ReferenceBody.set_size(1, 4);
    distConstraints[0]->ReferenceBody[0] = 'b';
    distConstraints[0]->ReferenceBody[1] = 'a';
    distConstraints[0]->ReferenceBody[2] = 's';
    distConstraints[0]->ReferenceBody[3] = 'e';
    distConstraints[0]->Bounds[0] = 0.0;
    distConstraints[0]->Bounds[1] = 100.0;
    distConstraints[0]->Weights = 0.0;
    //  Start disabled
    distConstraints[1] = gobj_4[1].init();
    distConstraints[1]->ReferenceBody.set_size(1, 4);
    distConstraints[1]->ReferenceBody[0] = 'b';
    distConstraints[1]->ReferenceBody[1] = 'a';
    distConstraints[1]->ReferenceBody[2] = 's';
    distConstraints[1]->ReferenceBody[3] = 'e';
    distConstraints[1]->Bounds[0] = 0.0;
    distConstraints[1]->Bounds[1] = 100.0;
    distConstraints[1]->Weights = 0.0;
    //  Start disabled
    distConstraints[2] = gobj_4[2].init();
    distConstraints[2]->ReferenceBody.set_size(1, 4);
    distConstraints[2]->ReferenceBody[0] = 'b';
    distConstraints[2]->ReferenceBody[1] = 'a';
    distConstraints[2]->ReferenceBody[2] = 's';
    distConstraints[2]->ReferenceBody[3] = 'e';
    distConstraints[2]->Bounds[0] = 0.0;
    distConstraints[2]->Bounds[1] = 100.0;
    distConstraints[2]->Weights = 0.0;
    //  Start disabled
    distConstraints[3] = gobj_4[3].init();
    distConstraints[3]->ReferenceBody.set_size(1, 4);
    distConstraints[3]->ReferenceBody[0] = 'b';
    distConstraints[3]->ReferenceBody[1] = 'a';
    distConstraints[3]->ReferenceBody[2] = 's';
    distConstraints[3]->ReferenceBody[3] = 'e';
    distConstraints[3]->Bounds[0] = 0.0;
    distConstraints[3]->Bounds[1] = 100.0;
    distConstraints[3]->Weights = 0.0;
    //  Start disabled
    distConstraints[4] = gobj_4[4].init();
    distConstraints[4]->ReferenceBody.set_size(1, 4);
    distConstraints[4]->ReferenceBody[0] = 'b';
    distConstraints[4]->ReferenceBody[1] = 'a';
    distConstraints[4]->ReferenceBody[2] = 's';
    distConstraints[4]->ReferenceBody[3] = 'e';
    distConstraints[4]->Bounds[0] = 0.0;
    distConstraints[4]->Bounds[1] = 100.0;
    distConstraints[4]->Weights = 0.0;
    //  Start disabled
    distConstraints[5] = gobj_4[5].init();
    distConstraints[5]->ReferenceBody.set_size(1, 4);
    distConstraints[5]->ReferenceBody[0] = 'b';
    distConstraints[5]->ReferenceBody[1] = 'a';
    distConstraints[5]->ReferenceBody[2] = 's';
    distConstraints[5]->ReferenceBody[3] = 'e';
    distConstraints[5]->Bounds[0] = 0.0;
    distConstraints[5]->Bounds[1] = 100.0;
    distConstraints[5]->Weights = 0.0;
    //  Start disabled
    distConstraints[6] = gobj_4[6].init();
    distConstraints[6]->ReferenceBody.set_size(1, 4);
    distConstraints[6]->ReferenceBody[0] = 'b';
    distConstraints[6]->ReferenceBody[1] = 'a';
    distConstraints[6]->ReferenceBody[2] = 's';
    distConstraints[6]->ReferenceBody[3] = 'e';
    distConstraints[6]->Bounds[0] = 0.0;
    distConstraints[6]->Bounds[1] = 100.0;
    distConstraints[6]->Weights = 0.0;
    //  Start disabled
    distConstraints[7] = gobj_4[7].init();
    distConstraints[7]->ReferenceBody.set_size(1, 4);
    distConstraints[7]->ReferenceBody[0] = 'b';
    distConstraints[7]->ReferenceBody[1] = 'a';
    distConstraints[7]->ReferenceBody[2] = 's';
    distConstraints[7]->ReferenceBody[3] = 'e';
    distConstraints[7]->Bounds[0] = 0.0;
    distConstraints[7]->Bounds[1] = 100.0;
    distConstraints[7]->Weights = 0.0;
    //  Start disabled
    distConstraints[8] = gobj_4[8].init();
    distConstraints[8]->ReferenceBody.set_size(1, 4);
    distConstraints[8]->ReferenceBody[0] = 'b';
    distConstraints[8]->ReferenceBody[1] = 'a';
    distConstraints[8]->ReferenceBody[2] = 's';
    distConstraints[8]->ReferenceBody[3] = 'e';
    distConstraints[8]->Bounds[0] = 0.0;
    distConstraints[8]->Bounds[1] = 100.0;
    distConstraints[8]->Weights = 0.0;
    //  Start disabled
    distConstraints[9] = gobj_4[9].init();
    distConstraints[9]->ReferenceBody.set_size(1, 4);
    distConstraints[9]->ReferenceBody[0] = 'b';
    distConstraints[9]->ReferenceBody[1] = 'a';
    distConstraints[9]->ReferenceBody[2] = 's';
    distConstraints[9]->ReferenceBody[3] = 'e';
    distConstraints[9]->Bounds[0] = 0.0;
    distConstraints[9]->Bounds[1] = 100.0;
    distConstraints[9]->Weights = 0.0;
    //  Start disabled
    distConstraints[10] = gobj_4[10].init();
    distConstraints[10]->ReferenceBody.set_size(1, 4);
    distConstraints[10]->ReferenceBody[0] = 'b';
    distConstraints[10]->ReferenceBody[1] = 'a';
    distConstraints[10]->ReferenceBody[2] = 's';
    distConstraints[10]->ReferenceBody[3] = 'e';
    distConstraints[10]->Bounds[0] = 0.0;
    distConstraints[10]->Bounds[1] = 100.0;
    distConstraints[10]->Weights = 0.0;
    //  Start disabled
    distConstraints[11] = gobj_4[11].init();
    distConstraints[11]->ReferenceBody.set_size(1, 4);
    distConstraints[11]->ReferenceBody[0] = 'b';
    distConstraints[11]->ReferenceBody[1] = 'a';
    distConstraints[11]->ReferenceBody[2] = 's';
    distConstraints[11]->ReferenceBody[3] = 'e';
    distConstraints[11]->Bounds[0] = 0.0;
    distConstraints[11]->Bounds[1] = 100.0;
    distConstraints[11]->Weights = 0.0;
    //  Start disabled
    distConstraints[12] = gobj_4[12].init();
    distConstraints[12]->ReferenceBody.set_size(1, 4);
    distConstraints[12]->ReferenceBody[0] = 'b';
    distConstraints[12]->ReferenceBody[1] = 'a';
    distConstraints[12]->ReferenceBody[2] = 's';
    distConstraints[12]->ReferenceBody[3] = 'e';
    distConstraints[12]->Bounds[0] = 0.0;
    distConstraints[12]->Bounds[1] = 100.0;
    distConstraints[12]->Weights = 0.0;
    //  Start disabled
    distConstraints[13] = gobj_4[13].init();
    distConstraints[13]->ReferenceBody.set_size(1, 4);
    distConstraints[13]->ReferenceBody[0] = 'b';
    distConstraints[13]->ReferenceBody[1] = 'a';
    distConstraints[13]->ReferenceBody[2] = 's';
    distConstraints[13]->ReferenceBody[3] = 'e';
    distConstraints[13]->Bounds[0] = 0.0;
    distConstraints[13]->Bounds[1] = 100.0;
    distConstraints[13]->Weights = 0.0;
    //  Start disabled
    distConstraints[14] = gobj_4[14].init();
    distConstraints[14]->ReferenceBody.set_size(1, 4);
    distConstraints[14]->ReferenceBody[0] = 'b';
    distConstraints[14]->ReferenceBody[1] = 'a';
    distConstraints[14]->ReferenceBody[2] = 's';
    distConstraints[14]->ReferenceBody[3] = 'e';
    distConstraints[14]->Bounds[0] = 0.0;
    distConstraints[14]->Bounds[1] = 100.0;
    distConstraints[14]->Weights = 0.0;
    //  Start disabled
    distConstraints[15] = gobj_4[15].init();
    distConstraints[15]->ReferenceBody.set_size(1, 4);
    distConstraints[15]->ReferenceBody[0] = 'b';
    distConstraints[15]->ReferenceBody[1] = 'a';
    distConstraints[15]->ReferenceBody[2] = 's';
    distConstraints[15]->ReferenceBody[3] = 'e';
    distConstraints[15]->Bounds[0] = 0.0;
    distConstraints[15]->Bounds[1] = 100.0;
    distConstraints[15]->Weights = 0.0;
    //  Start disabled
    distConstraints[16] = gobj_4[16].init();
    distConstraints[16]->ReferenceBody.set_size(1, 4);
    distConstraints[16]->ReferenceBody[0] = 'b';
    distConstraints[16]->ReferenceBody[1] = 'a';
    distConstraints[16]->ReferenceBody[2] = 's';
    distConstraints[16]->ReferenceBody[3] = 'e';
    distConstraints[16]->Bounds[0] = 0.0;
    distConstraints[16]->Bounds[1] = 100.0;
    distConstraints[16]->Weights = 0.0;
    //  Start disabled
    distConstraints[17] = gobj_4[17].init();
    distConstraints[17]->ReferenceBody.set_size(1, 4);
    distConstraints[17]->ReferenceBody[0] = 'b';
    distConstraints[17]->ReferenceBody[1] = 'a';
    distConstraints[17]->ReferenceBody[2] = 's';
    distConstraints[17]->ReferenceBody[3] = 'e';
    distConstraints[17]->Bounds[0] = 0.0;
    distConstraints[17]->Bounds[1] = 100.0;
    distConstraints[17]->Weights = 0.0;
    //  Start disabled
    distConstraints[18] = gobj_4[18].init();
    distConstraints[18]->ReferenceBody.set_size(1, 4);
    distConstraints[18]->ReferenceBody[0] = 'b';
    distConstraints[18]->ReferenceBody[1] = 'a';
    distConstraints[18]->ReferenceBody[2] = 's';
    distConstraints[18]->ReferenceBody[3] = 'e';
    distConstraints[18]->Bounds[0] = 0.0;
    distConstraints[18]->Bounds[1] = 100.0;
    distConstraints[18]->Weights = 0.0;
    //  Start disabled
    distConstraints[19] = gobj_4[19].init();
    distConstraints[19]->ReferenceBody.set_size(1, 4);
    distConstraints[19]->ReferenceBody[0] = 'b';
    distConstraints[19]->ReferenceBody[1] = 'a';
    distConstraints[19]->ReferenceBody[2] = 's';
    distConstraints[19]->ReferenceBody[3] = 'e';
    distConstraints[19]->Bounds[0] = 0.0;
    distConstraints[19]->Bounds[1] = 100.0;
    distConstraints[19]->Weights = 0.0;
    //  Start disabled
  }
  //  Update pose constraint
  std::copy(&targetPose[0], &targetPose[16],
            &poseConstraint.TargetTransform[0]);
  //  Update distance constraints
  for (int i{0}; i < 20; i++) {
    int b_i;
    //  Default to disabled
    lowerBound = 0.0;
    upperBound = 100.0;
    weight = 0.0;
    //  Enable if indices are valid and weight > 0
    b_i = distBodyIndices[i];
    if ((b_i >= 1) && (b_i <= 12) && (distRefBodyIndices[i] >= 1) &&
        (distRefBodyIndices[i] <= 12) && (distWeights[i] > 0.0)) {
      weight = distBoundsLower[i];
      if (weight > 0.0) {
        lowerBound = weight;
      }
      weight = distBoundsUpper[i];
      if ((weight > 0.0) && (weight <= 100.0)) {
        upperBound = weight;
      }
      weight = distWeights[i];
    }
    //  Update constraint (body names should stay constant for code generation)
    //  distConstraints{i}.BodyName = bodyName;  % Can't change after creation
    //  distConstraints{i}.ReferenceBody = refBodyName;  % Can't change after
    //  creation
    distConstraints[i]->Bounds[0] = lowerBound;
    distConstraints[i]->Bounds[1] = upperBound;
    distConstraints[i]->Weights = weight;
  }
  //  Solve IK with all constraints
  solver.step(qCurrent, poseConstraint, jointConstraint, distConstraints[0],
              distConstraints[1], distConstraints[2], distConstraints[3],
              distConstraints[4], distConstraints[5], distConstraints[6],
              distConstraints[7], distConstraints[8], distConstraints[9],
              distConstraints[10], distConstraints[11], distConstraints[12],
              distConstraints[13], distConstraints[14], distConstraints[15],
              distConstraints[16], distConstraints[17], distConstraints[18],
              distConstraints[19], qNext, solverInfo);
}

//
// Arguments    : void
// Return Type  : void
//
} // namespace codegen_inuse
} // namespace gik9dof
void solveGIKStepWrapper_delete()
{
  coder::robotics::manip::internal::DistanceBoundsConstraint *obj;
  coder::robotics::manip::internal::RigidBody *b_obj;
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
  for (int i{0}; i < 20; i++) {
    obj = &solver.Problem._pobj0[i];
    if (!obj->matlabCodegenIsDeleted) {
      obj->matlabCodegenIsDeleted = true;
    }
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
  for (int i{0}; i < 11; i++) {
    b_obj = &solver._pobj1[i];
    if (!b_obj->matlabCodegenIsDeleted) {
      b_obj->matlabCodegenIsDeleted = true;
    }
  }
  if (!solver._pobj3.Base.matlabCodegenIsDeleted) {
    solver._pobj3.Base.matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 11; i++) {
    b_obj = &solver._pobj3._pobj2[i];
    if (!b_obj->matlabCodegenIsDeleted) {
      b_obj->matlabCodegenIsDeleted = true;
    }
  }
  if (!robot.TreeInternal.Base.matlabCodegenIsDeleted) {
    robot.TreeInternal.Base.matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 11; i++) {
    b_obj = &robot.TreeInternal._pobj2[i];
    if (!b_obj->matlabCodegenIsDeleted) {
      b_obj->matlabCodegenIsDeleted = true;
    }
  }
  for (int i{0}; i < 11; i++) {
    b_obj = &gobj_7[i];
    if (!b_obj->matlabCodegenIsDeleted) {
      b_obj->matlabCodegenIsDeleted = true;
    }
  }
  for (int i{0}; i < 11; i++) {
    solver._pobj1[i]._pobj0.matlabCodegenDestructor();
  }
  for (int i{0}; i < 23; i++) {
    solver._pobj2[i].matlabCodegenDestructor();
  }
  solver._pobj3.Base._pobj0.matlabCodegenDestructor();
  solver._pobj3._pobj0[0].matlabCodegenDestructor();
  solver._pobj3._pobj0[1].matlabCodegenDestructor();
  for (int i{0}; i < 11; i++) {
    solver._pobj3._pobj2[i]._pobj0.matlabCodegenDestructor();
  }
  robot.TreeInternal.Base._pobj0.matlabCodegenDestructor();
  robot.TreeInternal._pobj0[0].matlabCodegenDestructor();
  robot.TreeInternal._pobj0[1].matlabCodegenDestructor();
  for (int i{0}; i < 11; i++) {
    robot.TreeInternal._pobj2[i]._pobj0.matlabCodegenDestructor();
  }
  robot._pobj0.matlabCodegenDestructor();
  for (int i{0}; i < 22; i++) {
    gobj_5[i].matlabCodegenDestructor();
  }
  for (int i{0}; i < 11; i++) {
    gobj_7[i]._pobj0.matlabCodegenDestructor();
  }
}

//
// Arguments    : void
// Return Type  : void
//
void solveGIKStepWrapper_init()
{
  solver_not_empty = false;
  bodyNames_not_empty = false;
}

//
// Arguments    : void
// Return Type  : void
//
void solveGIKStepWrapper_new()
{
  for (int i{0}; i < 11; i++) {
    gobj_7[i]._pobj0.matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 22; i++) {
    gobj_5[i].matlabCodegenIsDeleted = true;
  }
  robot._pobj0.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 11; i++) {
    robot.TreeInternal._pobj2[i]._pobj0.matlabCodegenIsDeleted = true;
  }
  robot.TreeInternal._pobj0[0].matlabCodegenIsDeleted = true;
  robot.TreeInternal._pobj0[1].matlabCodegenIsDeleted = true;
  robot.TreeInternal.Base._pobj0.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 11; i++) {
    solver._pobj3._pobj2[i]._pobj0.matlabCodegenIsDeleted = true;
  }
  solver._pobj3._pobj0[0].matlabCodegenIsDeleted = true;
  solver._pobj3._pobj0[1].matlabCodegenIsDeleted = true;
  solver._pobj3.Base._pobj0.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 23; i++) {
    solver._pobj2[i].matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 11; i++) {
    solver._pobj1[i]._pobj0.matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 11; i++) {
    gobj_7[i].matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 11; i++) {
    robot.TreeInternal._pobj2[i].matlabCodegenIsDeleted = true;
  }
  robot.TreeInternal.Base.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 11; i++) {
    solver._pobj3._pobj2[i].matlabCodegenIsDeleted = true;
  }
  solver._pobj3.Base.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 11; i++) {
    solver._pobj1[i].matlabCodegenIsDeleted = true;
  }
  robot.TreeInternal.matlabCodegenIsDeleted = true;
  solver._pobj3.matlabCodegenIsDeleted = true;
  solver.Problem._pobj2.matlabCodegenIsDeleted = true;
  solver.Problem._pobj1.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 20; i++) {
    solver.Problem._pobj0[i].matlabCodegenIsDeleted = true;
  }
  solver.Problem.matlabCodegenIsDeleted = true;
  solver._pobj4.matlabCodegenIsDeleted = true;
  solver.matlabCodegenIsDeleted = true;
  robot.matlabCodegenIsDeleted = true;
}

//
// File trailer for gik9dof_codegen_inuse_solveGIKStepWrapper.cpp
//
// [EOF]
//
