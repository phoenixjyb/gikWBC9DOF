//
// File: GIKSolver.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "GIKSolver.h"
#include "CoderTimeAPI.h"
#include "CollisionGeometry.h"
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
#include "eml_rand_mt19937ar_stateful.h"
#include "generalizedInverseKinematics.h"
#include "rigidBodyJoint.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"
#include "solveGIKStepWrapper_data.h"
#include "solveGIKStepWrapper_types.h"
#include "solveGIKStepWrapper_types1.h"
#include "coder_array.h"
#include "collisioncodegen_api.hpp"
#include "omp.h"
#include <algorithm>
#include <cstring>

// Function Declarations
namespace gik9dof {
static void solveGIKStepWrapper_delete(GIKSolver *aInstancePtr);

static void solveGIKStepWrapper_init(GIKSolver *aInstancePtr);

static void solveGIKStepWrapper_new(GIKSolver *aInstancePtr);

} // namespace gik9dof

// Function Definitions
//
// Arguments    : GIKSolver *aInstancePtr
// Return Type  : void
//
namespace gik9dof {
static void solveGIKStepWrapper_delete(GIKSolver *aInstancePtr)
{
  coder::robotics::manip::internal::CollisionGeometry d_obj;
  coder::robotics::manip::internal::CollisionSet *c_obj;
  coder::robotics::manip::internal::DistanceBoundsConstraint *obj;
  coder::robotics::manip::internal::RigidBody *b_obj;
  solveGIKStepWrapperStackData *localSD;
  double d;
  int i;
  int i1;
  localSD = aInstancePtr->getStackData();
  if (!localSD->pd->robot.matlabCodegenIsDeleted) {
    localSD->pd->robot.matlabCodegenIsDeleted = true;
  }
  if (!localSD->pd->solver.matlabCodegenIsDeleted) {
    localSD->pd->solver.matlabCodegenIsDeleted = true;
    if (localSD->pd->solver.isInitialized == 1) {
      localSD->pd->solver.isInitialized = 2;
    }
  }
  if (!localSD->pd->solver._pobj4.matlabCodegenIsDeleted) {
    localSD->pd->solver._pobj4.matlabCodegenIsDeleted = true;
  }
  if (!localSD->pd->solver.Problem.matlabCodegenIsDeleted) {
    localSD->pd->solver.Problem.matlabCodegenIsDeleted = true;
  }
  for (i = 0; i < 20; i++) {
    obj = &localSD->pd->solver.Problem._pobj0[i];
    if (!obj->matlabCodegenIsDeleted) {
      obj->matlabCodegenIsDeleted = true;
    }
  }
  if (!localSD->pd->solver.Problem._pobj1.matlabCodegenIsDeleted) {
    localSD->pd->solver.Problem._pobj1.matlabCodegenIsDeleted = true;
  }
  if (!localSD->pd->solver.Problem._pobj2.matlabCodegenIsDeleted) {
    localSD->pd->solver.Problem._pobj2.matlabCodegenIsDeleted = true;
  }
  if (!localSD->pd->solver._pobj3.matlabCodegenIsDeleted) {
    localSD->pd->solver._pobj3.matlabCodegenIsDeleted = true;
  }
  if (!localSD->pd->robot.TreeInternal.matlabCodegenIsDeleted) {
    localSD->pd->robot.TreeInternal.matlabCodegenIsDeleted = true;
  }
  for (i = 0; i < 11; i++) {
    b_obj = &localSD->pd->solver._pobj1[i];
    if (!b_obj->matlabCodegenIsDeleted) {
      b_obj->matlabCodegenIsDeleted = true;
    }
  }
  if (!localSD->pd->solver._pobj3.Base.matlabCodegenIsDeleted) {
    localSD->pd->solver._pobj3.Base.matlabCodegenIsDeleted = true;
  }
  for (i = 0; i < 11; i++) {
    b_obj = &localSD->pd->solver._pobj3._pobj2[i];
    if (!b_obj->matlabCodegenIsDeleted) {
      b_obj->matlabCodegenIsDeleted = true;
    }
  }
  if (!localSD->pd->robot.TreeInternal.Base.matlabCodegenIsDeleted) {
    localSD->pd->robot.TreeInternal.Base.matlabCodegenIsDeleted = true;
  }
  for (i = 0; i < 11; i++) {
    b_obj = &localSD->pd->robot.TreeInternal._pobj2[i];
    if (!b_obj->matlabCodegenIsDeleted) {
      b_obj->matlabCodegenIsDeleted = true;
    }
  }
  for (i = 0; i < 11; i++) {
    b_obj = &localSD->pd->gobj_7[i];
    if (!b_obj->matlabCodegenIsDeleted) {
      b_obj->matlabCodegenIsDeleted = true;
    }
  }
  for (i = 0; i < 11; i++) {
    c_obj = &localSD->pd->solver._pobj1[i]._pobj0;
    if (!c_obj->matlabCodegenIsDeleted) {
      c_obj->matlabCodegenIsDeleted = true;
      d = c_obj->Size;
      i1 = static_cast<int>(d);
      for (int b_i{0}; b_i < i1; b_i++) {
        d_obj = c_obj->CollisionGeometries[b_i];
        collisioncodegen_destructGeometry(&d_obj.CollisionPrimitive);
        c_obj->CollisionGeometries[b_i] = d_obj;
      }
    }
  }
  for (i = 0; i < 23; i++) {
    c_obj = &localSD->pd->solver._pobj2[i];
    if (!c_obj->matlabCodegenIsDeleted) {
      c_obj->matlabCodegenIsDeleted = true;
      d = c_obj->Size;
      i1 = static_cast<int>(d);
      for (int b_i{0}; b_i < i1; b_i++) {
        d_obj = c_obj->CollisionGeometries[b_i];
        collisioncodegen_destructGeometry(&d_obj.CollisionPrimitive);
        c_obj->CollisionGeometries[b_i] = d_obj;
      }
    }
  }
  if (!localSD->pd->solver._pobj3.Base._pobj0.matlabCodegenIsDeleted) {
    localSD->pd->solver._pobj3.Base._pobj0.matlabCodegenIsDeleted = true;
    d = localSD->pd->solver._pobj3.Base._pobj0.Size;
    i = static_cast<int>(d);
    for (int b_i{0}; b_i < i; b_i++) {
      d_obj = localSD->pd->solver._pobj3.Base._pobj0.CollisionGeometries[b_i];
      collisioncodegen_destructGeometry(&d_obj.CollisionPrimitive);
      localSD->pd->solver._pobj3.Base._pobj0.CollisionGeometries[b_i] = d_obj;
    }
  }
  for (i = 0; i < 2; i++) {
    c_obj = &localSD->pd->solver._pobj3._pobj0[i];
    if (!c_obj->matlabCodegenIsDeleted) {
      c_obj->matlabCodegenIsDeleted = true;
      d = c_obj->Size;
      i1 = static_cast<int>(d);
      for (int b_i{0}; b_i < i1; b_i++) {
        d_obj = c_obj->CollisionGeometries[b_i];
        collisioncodegen_destructGeometry(&d_obj.CollisionPrimitive);
        c_obj->CollisionGeometries[b_i] = d_obj;
      }
    }
  }
  for (i = 0; i < 11; i++) {
    c_obj = &localSD->pd->solver._pobj3._pobj2[i]._pobj0;
    if (!c_obj->matlabCodegenIsDeleted) {
      c_obj->matlabCodegenIsDeleted = true;
      d = c_obj->Size;
      i1 = static_cast<int>(d);
      for (int b_i{0}; b_i < i1; b_i++) {
        d_obj = c_obj->CollisionGeometries[b_i];
        collisioncodegen_destructGeometry(&d_obj.CollisionPrimitive);
        c_obj->CollisionGeometries[b_i] = d_obj;
      }
    }
  }
  if (!localSD->pd->robot.TreeInternal.Base._pobj0.matlabCodegenIsDeleted) {
    localSD->pd->robot.TreeInternal.Base._pobj0.matlabCodegenIsDeleted = true;
    d = localSD->pd->robot.TreeInternal.Base._pobj0.Size;
    i = static_cast<int>(d);
    for (int b_i{0}; b_i < i; b_i++) {
      d_obj =
          localSD->pd->robot.TreeInternal.Base._pobj0.CollisionGeometries[b_i];
      collisioncodegen_destructGeometry(&d_obj.CollisionPrimitive);
      localSD->pd->robot.TreeInternal.Base._pobj0.CollisionGeometries[b_i] =
          d_obj;
    }
  }
  for (i = 0; i < 2; i++) {
    c_obj = &localSD->pd->robot.TreeInternal._pobj0[i];
    if (!c_obj->matlabCodegenIsDeleted) {
      c_obj->matlabCodegenIsDeleted = true;
      d = c_obj->Size;
      i1 = static_cast<int>(d);
      for (int b_i{0}; b_i < i1; b_i++) {
        d_obj = c_obj->CollisionGeometries[b_i];
        collisioncodegen_destructGeometry(&d_obj.CollisionPrimitive);
        c_obj->CollisionGeometries[b_i] = d_obj;
      }
    }
  }
  for (i = 0; i < 11; i++) {
    c_obj = &localSD->pd->robot.TreeInternal._pobj2[i]._pobj0;
    if (!c_obj->matlabCodegenIsDeleted) {
      c_obj->matlabCodegenIsDeleted = true;
      d = c_obj->Size;
      i1 = static_cast<int>(d);
      for (int b_i{0}; b_i < i1; b_i++) {
        d_obj = c_obj->CollisionGeometries[b_i];
        collisioncodegen_destructGeometry(&d_obj.CollisionPrimitive);
        c_obj->CollisionGeometries[b_i] = d_obj;
      }
    }
  }
  if (!localSD->pd->robot._pobj0.matlabCodegenIsDeleted) {
    localSD->pd->robot._pobj0.matlabCodegenIsDeleted = true;
    d = localSD->pd->robot._pobj0.Size;
    i = static_cast<int>(d);
    for (int b_i{0}; b_i < i; b_i++) {
      d_obj = localSD->pd->robot._pobj0.CollisionGeometries[b_i];
      collisioncodegen_destructGeometry(&d_obj.CollisionPrimitive);
      localSD->pd->robot._pobj0.CollisionGeometries[b_i] = d_obj;
    }
  }
  for (i = 0; i < 22; i++) {
    c_obj = &localSD->pd->gobj_5[i];
    if (!c_obj->matlabCodegenIsDeleted) {
      c_obj->matlabCodegenIsDeleted = true;
      d = c_obj->Size;
      i1 = static_cast<int>(d);
      for (int b_i{0}; b_i < i1; b_i++) {
        d_obj = c_obj->CollisionGeometries[b_i];
        collisioncodegen_destructGeometry(&d_obj.CollisionPrimitive);
        c_obj->CollisionGeometries[b_i] = d_obj;
      }
    }
  }
  for (i = 0; i < 11; i++) {
    c_obj = &localSD->pd->gobj_7[i]._pobj0;
    if (!c_obj->matlabCodegenIsDeleted) {
      c_obj->matlabCodegenIsDeleted = true;
      d = c_obj->Size;
      i1 = static_cast<int>(d);
      for (int b_i{0}; b_i < i1; b_i++) {
        d_obj = c_obj->CollisionGeometries[b_i];
        collisioncodegen_destructGeometry(&d_obj.CollisionPrimitive);
        c_obj->CollisionGeometries[b_i] = d_obj;
      }
    }
  }
}

//
// Arguments    : GIKSolver *aInstancePtr
// Return Type  : void
//
static void solveGIKStepWrapper_init(GIKSolver *aInstancePtr)
{
  solveGIKStepWrapperStackData *localSD;
  localSD = aInstancePtr->getStackData();
  localSD->pd->solver_not_empty = false;
}

//
// Arguments    : GIKSolver *aInstancePtr
// Return Type  : void
//
static void solveGIKStepWrapper_new(GIKSolver *aInstancePtr)
{
  solveGIKStepWrapperStackData *localSD;
  localSD = aInstancePtr->getStackData();
  for (int i{0}; i < 11; i++) {
    localSD->pd->gobj_7[i]._pobj0.matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 22; i++) {
    localSD->pd->gobj_5[i].matlabCodegenIsDeleted = true;
  }
  localSD->pd->robot._pobj0.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 11; i++) {
    localSD->pd->robot.TreeInternal._pobj2[i]._pobj0.matlabCodegenIsDeleted =
        true;
  }
  localSD->pd->robot.TreeInternal._pobj0[0].matlabCodegenIsDeleted = true;
  localSD->pd->robot.TreeInternal._pobj0[1].matlabCodegenIsDeleted = true;
  localSD->pd->robot.TreeInternal.Base._pobj0.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 11; i++) {
    localSD->pd->solver._pobj3._pobj2[i]._pobj0.matlabCodegenIsDeleted = true;
  }
  localSD->pd->solver._pobj3._pobj0[0].matlabCodegenIsDeleted = true;
  localSD->pd->solver._pobj3._pobj0[1].matlabCodegenIsDeleted = true;
  localSD->pd->solver._pobj3.Base._pobj0.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 23; i++) {
    localSD->pd->solver._pobj2[i].matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 11; i++) {
    localSD->pd->solver._pobj1[i]._pobj0.matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 11; i++) {
    localSD->pd->gobj_7[i].matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 11; i++) {
    localSD->pd->robot.TreeInternal._pobj2[i].matlabCodegenIsDeleted = true;
  }
  localSD->pd->robot.TreeInternal.Base.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 11; i++) {
    localSD->pd->solver._pobj3._pobj2[i].matlabCodegenIsDeleted = true;
  }
  localSD->pd->solver._pobj3.Base.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 11; i++) {
    localSD->pd->solver._pobj1[i].matlabCodegenIsDeleted = true;
  }
  localSD->pd->robot.TreeInternal.matlabCodegenIsDeleted = true;
  localSD->pd->solver._pobj3.matlabCodegenIsDeleted = true;
  localSD->pd->solver.Problem._pobj2.matlabCodegenIsDeleted = true;
  localSD->pd->solver.Problem._pobj1.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 20; i++) {
    localSD->pd->solver.Problem._pobj0[i].matlabCodegenIsDeleted = true;
  }
  localSD->pd->solver.Problem.matlabCodegenIsDeleted = true;
  localSD->pd->solver._pobj4.matlabCodegenIsDeleted = true;
  localSD->pd->solver.matlabCodegenIsDeleted = true;
  localSD->pd->robot.matlabCodegenIsDeleted = true;
}

//
// Arguments    : void
// Return Type  : void
//
GIKSolver::GIKSolver()
{
  SD_.pd = &pd_;
  omp_init_nest_lock(&solveGIKStepWrapper_nestLockGlobal);
  solveGIKStepWrapper_new(this);
  solveGIKStepWrapper_init(this);
  eml_rand_mt19937ar_stateful_init(this);
  CoderTimeAPI::callCoderClockGettime_init(this);
}

//
// Arguments    : void
// Return Type  : void
//
GIKSolver::~GIKSolver()
{
  solveGIKStepWrapper_delete(this);
  omp_destroy_nest_lock(&solveGIKStepWrapper_nestLockGlobal);
}

//
// Arguments    : void
// Return Type  : solveGIKStepWrapperStackData *
//
solveGIKStepWrapperStackData *GIKSolver::getStackData()
{
  return &SD_;
}

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
void GIKSolver::solveGIKStepWrapper(const double qCurrent[9],
                                    const double targetPose[16], const int[20],
                                    const int[20],
                                    const double distBoundsLower[20],
                                    const double distBoundsUpper[20],
                                    const double distWeights[20],
                                    double qNext[9], struct0_T *solverInfo)
{
  coder::constraintDistanceBounds *r;
  ::coder::array<double, 2U> defaultValues_f2;
  ::coder::array<double, 2U> limits;
  double lowerBound;
  double params_DampingBias;
  double params_ErrorChangeTolerance;
  double params_MaxNumIteration;
  double params_StepTolerance;
  double upperBound;
  double weight;
  int loop_ub;
  boolean_T b_expl_temp;
  boolean_T params_RandomRestart;
  boolean_T params_UseErrorDamping;
  //  Not used (body pairs are fixed)
  //  Persistent solver and constraints
  if (!pd_.solver_not_empty) {
    int b_loop_ub;
    char expl_temp[18];
    //  Build robot model procedurally
    codegen_inuse::buildRobotForCodegen(this, pd_.gobj_5[0], pd_.gobj_6[0],
                                        pd_.gobj_7[0], pd_.robot);
    //  Create GIK solver with 22 constraint inputs (1 pose + 1 joint + 20
    //  distance)
    pd_.solver.init(this, pd_.robot);
    pd_.solver_not_empty = true;
    //  Configure solver parameters for validation testing
    params_MaxNumIteration = pd_.solver.Solver->getSolverParams(
        expl_temp, weight, lowerBound, upperBound, b_expl_temp,
        params_RandomRestart, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    b_expl_temp = pd_.solver.EnforceJointLimits;
    pd_.solver.set_SolverParameters(
        params_MaxNumIteration, 10.0, lowerBound, upperBound, b_expl_temp,
        params_RandomRestart, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    //  10 seconds - generous for validation
    pd_.solver.Solver->getSolverParams(
        expl_temp, weight, lowerBound, upperBound, b_expl_temp,
        params_RandomRestart, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    b_expl_temp = pd_.solver.EnforceJointLimits;
    pd_.solver.set_SolverParameters(
        1000.0, weight, lowerBound, upperBound, b_expl_temp,
        params_RandomRestart, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    //  Increased from 50 for better convergence
    params_MaxNumIteration = pd_.solver.Solver->getSolverParams(
        expl_temp, weight, lowerBound, upperBound, b_expl_temp,
        params_RandomRestart, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    b_expl_temp = pd_.solver.EnforceJointLimits;
    pd_.solver.set_SolverParameters(
        params_MaxNumIteration, weight, lowerBound, upperBound, b_expl_temp,
        false, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    params_MaxNumIteration = pd_.solver.Solver->getSolverParams(
        expl_temp, weight, lowerBound, upperBound, b_expl_temp,
        params_RandomRestart, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    b_expl_temp = pd_.solver.EnforceJointLimits;
    pd_.solver.set_SolverParameters(
        params_MaxNumIteration, weight, lowerBound, 1.0E-6, b_expl_temp,
        params_RandomRestart, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    params_MaxNumIteration = pd_.solver.Solver->getSolverParams(
        expl_temp, weight, lowerBound, upperBound, b_expl_temp,
        params_RandomRestart, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    b_expl_temp = pd_.solver.EnforceJointLimits;
    pd_.solver.set_SolverParameters(
        params_MaxNumIteration, weight, 1.0E-7, upperBound, b_expl_temp,
        params_RandomRestart, params_StepTolerance, params_ErrorChangeTolerance,
        params_DampingBias, params_UseErrorDamping);
    //  Create pose and joint constraints
    for (int i{0}; i < 16; i++) {
      pd_.poseConstraint.ConstructorPropertyDefaultValues.f2[i] = iv[i];
    }
    pd_.poseConstraint.ConstructorPropertyDefaultValues.f3 = 0.0;
    pd_.poseConstraint.ConstructorPropertyDefaultValues.f4 = 0.0;
    pd_.poseConstraint.ConstructorPropertyDefaultValues.f5[0] = 1.0;
    pd_.poseConstraint.ConstructorPropertyDefaultValues.f5[1] = 1.0;
    for (int i{0}; i < 17; i++) {
      pd_.poseConstraint.EndEffector[i] = cv[i];
    }
    upperBound = pd_.poseConstraint.ConstructorPropertyDefaultValues.f3;
    weight = pd_.poseConstraint.ConstructorPropertyDefaultValues.f4;
    params_MaxNumIteration =
        pd_.poseConstraint.ConstructorPropertyDefaultValues.f5[0];
    lowerBound = pd_.poseConstraint.ConstructorPropertyDefaultValues.f5[1];
    std::copy(&pd_.poseConstraint.ConstructorPropertyDefaultValues.f2[0],
              &pd_.poseConstraint.ConstructorPropertyDefaultValues.f2[16],
              &pd_.poseConstraint.TargetTransform[0]);
    pd_.poseConstraint.OrientationTolerance = upperBound;
    pd_.poseConstraint.PositionTolerance = weight;
    pd_.poseConstraint.Weights[0] = params_MaxNumIteration;
    pd_.poseConstraint.Weights[1] = lowerBound;
    pd_.robot.TreeInternal.get_JointPositionLimits(limits);
    pd_.jointConstraint.BoundsInternal.set_size(limits.size(0), 2);
    loop_ub = limits.size(0) << 1;
    for (int i{0}; i < loop_ub; i++) {
      pd_.jointConstraint.BoundsInternal[i] = limits[i];
    }
    pd_.jointConstraint.NumElements =
        pd_.jointConstraint.BoundsInternal.size(0);
    loop_ub = static_cast<int>(pd_.jointConstraint.NumElements);
    pd_.jointConstraint.WeightsInternal.set_size(1, loop_ub);
    for (int i{0}; i < loop_ub; i++) {
      pd_.jointConstraint.WeightsInternal[i] = 1.0;
    }
    loop_ub = pd_.jointConstraint.WeightsInternal.size(1);
    defaultValues_f2.set_size(1, loop_ub);
    b_loop_ub = pd_.jointConstraint.WeightsInternal.size(1);
    for (int i{0}; i < b_loop_ub; i++) {
      defaultValues_f2[i] = pd_.jointConstraint.WeightsInternal[i];
    }
    pd_.jointConstraint.ConstructorPropertyDefaultValues.f1.set_size(
        pd_.jointConstraint.BoundsInternal.size(0), 2);
    b_loop_ub = pd_.jointConstraint.BoundsInternal.size(0) << 1;
    for (int i{0}; i < b_loop_ub; i++) {
      pd_.jointConstraint.ConstructorPropertyDefaultValues.f1[i] =
          pd_.jointConstraint.BoundsInternal[i];
    }
    pd_.jointConstraint.ConstructorPropertyDefaultValues.f2.set_size(1,
                                                                     loop_ub);
    for (int i{0}; i < loop_ub; i++) {
      pd_.jointConstraint.ConstructorPropertyDefaultValues.f2[i] =
          defaultValues_f2[i];
    }
    pd_.jointConstraint.BoundsInternal.set_size(
        pd_.jointConstraint.ConstructorPropertyDefaultValues.f1.size(0), 2);
    loop_ub = pd_.jointConstraint.ConstructorPropertyDefaultValues.f1.size(0)
              << 1;
    for (int i{0}; i < loop_ub; i++) {
      pd_.jointConstraint.BoundsInternal[i] =
          pd_.jointConstraint.ConstructorPropertyDefaultValues.f1[i];
    }
    loop_ub = pd_.jointConstraint.ConstructorPropertyDefaultValues.f2.size(1);
    defaultValues_f2.set_size(1, loop_ub);
    b_loop_ub = pd_.jointConstraint.ConstructorPropertyDefaultValues.f2.size(1);
    for (int i{0}; i < b_loop_ub; i++) {
      defaultValues_f2[i] =
          pd_.jointConstraint.ConstructorPropertyDefaultValues.f2[i];
    }
    pd_.jointConstraint.WeightsInternal.set_size(1, loop_ub);
    for (int i{0}; i < loop_ub; i++) {
      pd_.jointConstraint.WeightsInternal[i] = defaultValues_f2[i];
    }
    //  Create 20 distance constraints with FIXED body pairs
    //  For code generation, body names must be compile-time constants
    //  All constraints start disabled (weight=0) and are enabled via weights
    //  parameter Gripper to various bodies (most common use case)
    pd_.distConstraints[0] = pd_.gobj_4[0].init();
    pd_.distConstraints[0]->ReferenceBody.set_size(1, 21);
    r = pd_.distConstraints[0];
    for (int i{0}; i < 21; i++) {
      r->ReferenceBody[i] = cv1[i];
    }
    pd_.distConstraints[1] = pd_.gobj_4[1].init();
    pd_.distConstraints[1]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[1]->ReferenceBody[0] = 'b';
    pd_.distConstraints[1]->ReferenceBody[1] = 'a';
    pd_.distConstraints[1]->ReferenceBody[2] = 's';
    pd_.distConstraints[1]->ReferenceBody[3] = 'e';
    pd_.distConstraints[2] = pd_.gobj_4[2].init();
    pd_.distConstraints[2]->ReferenceBody.set_size(1, 18);
    r = pd_.distConstraints[2];
    for (int i{0}; i < 18; i++) {
      r->ReferenceBody[i] = cv2[i];
    }
    pd_.gobj_4[3].ConstructorPropertyDefaultValues.f2[0] = 0.0;
    pd_.gobj_4[3].ConstructorPropertyDefaultValues.f2[1] = 0.0;
    pd_.gobj_4[3].ConstructorPropertyDefaultValues.f3 = 1.0;
    pd_.gobj_4[3].EndEffector.set_size(1, 14);
    for (int i{0}; i < 14; i++) {
      pd_.gobj_4[3].EndEffector[i] = cv3[i];
    }
    upperBound = pd_.gobj_4[3].ConstructorPropertyDefaultValues.f3;
    pd_.gobj_4[3].ReferenceBody.set_size(0, 0);
    params_StepTolerance = pd_.gobj_4[3].ConstructorPropertyDefaultValues.f2[0];
    pd_.gobj_4[3].Bounds[0] = params_StepTolerance;
    params_StepTolerance = pd_.gobj_4[3].ConstructorPropertyDefaultValues.f2[1];
    pd_.gobj_4[3].Bounds[1] = params_StepTolerance;
    pd_.gobj_4[3].Weights = upperBound;
    pd_.distConstraints[3] = &pd_.gobj_4[3];
    pd_.distConstraints[3]->ReferenceBody.set_size(1, 21);
    r = pd_.distConstraints[3];
    for (int i{0}; i < 21; i++) {
      r->ReferenceBody[i] = cv1[i];
    }
    pd_.gobj_4[4].ConstructorPropertyDefaultValues.f2[0] = 0.0;
    pd_.gobj_4[4].ConstructorPropertyDefaultValues.f2[1] = 0.0;
    pd_.gobj_4[4].ConstructorPropertyDefaultValues.f3 = 1.0;
    pd_.gobj_4[4].EndEffector.set_size(1, 14);
    for (int i{0}; i < 14; i++) {
      pd_.gobj_4[4].EndEffector[i] = cv4[i];
    }
    upperBound = pd_.gobj_4[4].ConstructorPropertyDefaultValues.f3;
    pd_.gobj_4[4].ReferenceBody.set_size(0, 0);
    params_StepTolerance = pd_.gobj_4[4].ConstructorPropertyDefaultValues.f2[0];
    pd_.gobj_4[4].Bounds[0] = params_StepTolerance;
    params_StepTolerance = pd_.gobj_4[4].ConstructorPropertyDefaultValues.f2[1];
    pd_.gobj_4[4].Bounds[1] = params_StepTolerance;
    pd_.gobj_4[4].Weights = upperBound;
    pd_.distConstraints[4] = &pd_.gobj_4[4];
    pd_.distConstraints[4]->ReferenceBody.set_size(1, 21);
    r = pd_.distConstraints[4];
    for (int i{0}; i < 21; i++) {
      r->ReferenceBody[i] = cv1[i];
    }
    //  Remaining constraints with default gripper->base pairs
    pd_.distConstraints[5] = pd_.gobj_4[5].init();
    pd_.distConstraints[5]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[5]->ReferenceBody[0] = 'b';
    pd_.distConstraints[5]->ReferenceBody[1] = 'a';
    pd_.distConstraints[5]->ReferenceBody[2] = 's';
    pd_.distConstraints[5]->ReferenceBody[3] = 'e';
    pd_.distConstraints[6] = pd_.gobj_4[6].init();
    pd_.distConstraints[6]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[6]->ReferenceBody[0] = 'b';
    pd_.distConstraints[6]->ReferenceBody[1] = 'a';
    pd_.distConstraints[6]->ReferenceBody[2] = 's';
    pd_.distConstraints[6]->ReferenceBody[3] = 'e';
    pd_.distConstraints[7] = pd_.gobj_4[7].init();
    pd_.distConstraints[7]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[7]->ReferenceBody[0] = 'b';
    pd_.distConstraints[7]->ReferenceBody[1] = 'a';
    pd_.distConstraints[7]->ReferenceBody[2] = 's';
    pd_.distConstraints[7]->ReferenceBody[3] = 'e';
    pd_.distConstraints[8] = pd_.gobj_4[8].init();
    pd_.distConstraints[8]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[8]->ReferenceBody[0] = 'b';
    pd_.distConstraints[8]->ReferenceBody[1] = 'a';
    pd_.distConstraints[8]->ReferenceBody[2] = 's';
    pd_.distConstraints[8]->ReferenceBody[3] = 'e';
    pd_.distConstraints[9] = pd_.gobj_4[9].init();
    pd_.distConstraints[9]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[9]->ReferenceBody[0] = 'b';
    pd_.distConstraints[9]->ReferenceBody[1] = 'a';
    pd_.distConstraints[9]->ReferenceBody[2] = 's';
    pd_.distConstraints[9]->ReferenceBody[3] = 'e';
    pd_.distConstraints[10] = pd_.gobj_4[10].init();
    pd_.distConstraints[10]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[10]->ReferenceBody[0] = 'b';
    pd_.distConstraints[10]->ReferenceBody[1] = 'a';
    pd_.distConstraints[10]->ReferenceBody[2] = 's';
    pd_.distConstraints[10]->ReferenceBody[3] = 'e';
    pd_.distConstraints[11] = pd_.gobj_4[11].init();
    pd_.distConstraints[11]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[11]->ReferenceBody[0] = 'b';
    pd_.distConstraints[11]->ReferenceBody[1] = 'a';
    pd_.distConstraints[11]->ReferenceBody[2] = 's';
    pd_.distConstraints[11]->ReferenceBody[3] = 'e';
    pd_.distConstraints[12] = pd_.gobj_4[12].init();
    pd_.distConstraints[12]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[12]->ReferenceBody[0] = 'b';
    pd_.distConstraints[12]->ReferenceBody[1] = 'a';
    pd_.distConstraints[12]->ReferenceBody[2] = 's';
    pd_.distConstraints[12]->ReferenceBody[3] = 'e';
    pd_.distConstraints[13] = pd_.gobj_4[13].init();
    pd_.distConstraints[13]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[13]->ReferenceBody[0] = 'b';
    pd_.distConstraints[13]->ReferenceBody[1] = 'a';
    pd_.distConstraints[13]->ReferenceBody[2] = 's';
    pd_.distConstraints[13]->ReferenceBody[3] = 'e';
    pd_.distConstraints[14] = pd_.gobj_4[14].init();
    pd_.distConstraints[14]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[14]->ReferenceBody[0] = 'b';
    pd_.distConstraints[14]->ReferenceBody[1] = 'a';
    pd_.distConstraints[14]->ReferenceBody[2] = 's';
    pd_.distConstraints[14]->ReferenceBody[3] = 'e';
    pd_.distConstraints[15] = pd_.gobj_4[15].init();
    pd_.distConstraints[15]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[15]->ReferenceBody[0] = 'b';
    pd_.distConstraints[15]->ReferenceBody[1] = 'a';
    pd_.distConstraints[15]->ReferenceBody[2] = 's';
    pd_.distConstraints[15]->ReferenceBody[3] = 'e';
    pd_.distConstraints[16] = pd_.gobj_4[16].init();
    pd_.distConstraints[16]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[16]->ReferenceBody[0] = 'b';
    pd_.distConstraints[16]->ReferenceBody[1] = 'a';
    pd_.distConstraints[16]->ReferenceBody[2] = 's';
    pd_.distConstraints[16]->ReferenceBody[3] = 'e';
    pd_.distConstraints[17] = pd_.gobj_4[17].init();
    pd_.distConstraints[17]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[17]->ReferenceBody[0] = 'b';
    pd_.distConstraints[17]->ReferenceBody[1] = 'a';
    pd_.distConstraints[17]->ReferenceBody[2] = 's';
    pd_.distConstraints[17]->ReferenceBody[3] = 'e';
    pd_.distConstraints[18] = pd_.gobj_4[18].init();
    pd_.distConstraints[18]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[18]->ReferenceBody[0] = 'b';
    pd_.distConstraints[18]->ReferenceBody[1] = 'a';
    pd_.distConstraints[18]->ReferenceBody[2] = 's';
    pd_.distConstraints[18]->ReferenceBody[3] = 'e';
    pd_.distConstraints[19] = pd_.gobj_4[19].init();
    pd_.distConstraints[19]->ReferenceBody.set_size(1, 4);
    pd_.distConstraints[19]->ReferenceBody[0] = 'b';
    pd_.distConstraints[19]->ReferenceBody[1] = 'a';
    pd_.distConstraints[19]->ReferenceBody[2] = 's';
    pd_.distConstraints[19]->ReferenceBody[3] = 'e';
    //  Initialize all as disabled
    for (loop_ub = 0; loop_ub < 20; loop_ub++) {
      r = pd_.distConstraints[loop_ub];
      r->Bounds[0] = 0.0;
      r->Bounds[1] = 100.0;
      pd_.distConstraints[loop_ub]->Weights = 0.0;
    }
  }
  //  Update pose constraint
  std::copy(&targetPose[0], &targetPose[16],
            &pd_.poseConstraint.TargetTransform[0]);
  //  Update distance constraint bounds and weights (body pairs are FIXED at
  //  initialization)
  for (loop_ub = 0; loop_ub < 20; loop_ub++) {
    lowerBound = 0.0;
    upperBound = 100.0;
    weight = 0.0;
    //  Process bounds if this constraint is enabled
    params_StepTolerance = distWeights[loop_ub];
    if (params_StepTolerance > 0.0) {
      params_MaxNumIteration = distBoundsLower[loop_ub];
      if (params_MaxNumIteration > 0.0) {
        lowerBound = params_MaxNumIteration;
      }
      params_MaxNumIteration = distBoundsUpper[loop_ub];
      if ((params_MaxNumIteration > 0.0) && (params_MaxNumIteration <= 100.0)) {
        upperBound = params_MaxNumIteration;
      }
      weight = params_StepTolerance;
    }
    pd_.distConstraints[loop_ub]->Bounds[0] = lowerBound;
    pd_.distConstraints[loop_ub]->Bounds[1] = upperBound;
    pd_.distConstraints[loop_ub]->Weights = weight;
  }
  //  Solve IK with all constraints
  std::copy(&qCurrent[0], &qCurrent[9], &qNext[0]);
  pd_.solver.step(
      this, qNext, pd_.poseConstraint, pd_.jointConstraint,
      pd_.distConstraints[0], pd_.distConstraints[1], pd_.distConstraints[2],
      pd_.distConstraints[3], pd_.distConstraints[4], pd_.distConstraints[5],
      pd_.distConstraints[6], pd_.distConstraints[7], pd_.distConstraints[8],
      pd_.distConstraints[9], pd_.distConstraints[10], pd_.distConstraints[11],
      pd_.distConstraints[12], pd_.distConstraints[13], pd_.distConstraints[14],
      pd_.distConstraints[15], pd_.distConstraints[16], pd_.distConstraints[17],
      pd_.distConstraints[18], pd_.distConstraints[19], solverInfo);
}

} // namespace gik9dof

//
// File trailer for GIKSolver.cpp
//
// [EOF]
//
