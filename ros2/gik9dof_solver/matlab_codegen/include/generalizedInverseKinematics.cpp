//
// File: generalizedInverseKinematics.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

// Include Files
#include "generalizedInverseKinematics.h"
#include "CharacterVector.h"
#include "CollisionGeometry.h"
#include "CollisionSet.h"
#include "DistanceBoundsConstraint.h"
#include "ErrorDampedLevenbergMarquardt.h"
#include "GIKProblem.h"
#include "JointPositionBounds.h"
#include "PoseTarget.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "SystemTimeProvider.h"
#include "constraintDistanceBounds.h"
#include "constraintJointBounds.h"
#include "constraintPoseTarget.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_data.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_internal_types.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "ixfun.h"
#include "rigidBodyJoint.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"
#include "tic.h"
#include "toc.h"
#include "coder_array.h"
#include "coder_bounded_array.h"
#include "coder_posix_time.h"
#include "collisioncodegen_api.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

// Variable Definitions
static const char cv13[18]{'L', 'e', 'v', 'e', 'n', 'b', 'e', 'r', 'g',
                           'M', 'a', 'r', 'q', 'u', 'a', 'r', 'd', 't'};

// Function Definitions
//
// Arguments    : void
// Return Type  : generalizedInverseKinematics
//
namespace coder {
generalizedInverseKinematics::generalizedInverseKinematics()
{
  matlabCodegenIsDeleted = true;
}

//
// Arguments    : void
// Return Type  : void
//
generalizedInverseKinematics::~generalizedInverseKinematics()
{
  matlabCodegenDestructor();
}

//
// Arguments    : rigidBodyTree &varargin_2
// Return Type  : generalizedInverseKinematics *
//
generalizedInverseKinematics *
generalizedInverseKinematics::init(rigidBodyTree &varargin_2)
{
  void *copyGeometryInternal;
  generalizedInverseKinematics *obj;
  robotics::manip::internal::CharacterVector b_obj;
  robotics::manip::internal::CharacterVector c_obj;
  robotics::manip::internal::CollisionGeometry b_newObj;
  robotics::manip::internal::CollisionGeometry e_obj;
  robotics::manip::internal::CollisionSet *d_obj;
  robotics::manip::internal::CollisionSet *newObj;
  robotics::manip::internal::RigidBody *body;
  robotics::manip::internal::RigidBody *parent;
  robotics::manip::internal::RigidBodyTree *newrobot;
  double bid;
  int obj_size[2];
  int loop_ub;
  char obj_data[200];
  obj = this;
  obj->EnforceJointLimits = true;
  obj->isInitialized = 0;
  newrobot = obj->_pobj3.init();
  b_obj = varargin_2.TreeInternal.Base.NameInternal;
  if (b_obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(b_obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &obj_data[0]);
  }
  bid = newrobot->findBodyIndexByName(obj_data, obj_size);
  if ((!(bid == 0.0)) && (bid < 0.0)) {
    c_obj = newrobot->Base.NameInternal;
    c_obj.Length = loop_ub;
    if (loop_ub < 1) {
      loop_ub = 0;
    }
    if (loop_ub - 1 >= 0) {
      ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &c_obj.Vector[0]);
    }
    newrobot->Base.NameInternal = c_obj;
  }
  d_obj = varargin_2.TreeInternal.Base.CollisionsInternal;
  newObj = obj->_pobj2[0].init(d_obj->MaxElements);
  newObj->Size = d_obj->Size;
  bid = d_obj->Size;
  loop_ub = static_cast<int>(bid);
  for (int i{0}; i < loop_ub; i++) {
    e_obj = d_obj->CollisionGeometries[i];
    copyGeometryInternal =
        collisioncodegen_copyGeometry(e_obj.CollisionPrimitive);
    b_newObj.CollisionPrimitive = copyGeometryInternal;
    ::std::copy(&e_obj.LocalPose[0], &e_obj.LocalPose[16],
                &b_newObj.LocalPose[0]);
    b_newObj.MeshScale[0] = e_obj.MeshScale[0];
    b_newObj.MeshScale[1] = e_obj.MeshScale[1];
    b_newObj.MeshScale[2] = e_obj.MeshScale[2];
    ::std::copy(&e_obj.WorldPose[0], &e_obj.WorldPose[16],
                &b_newObj.WorldPose[0]);
    newObj->CollisionGeometries[i] = b_newObj;
  }
  double g_idx_1;
  double g_idx_2;
  newrobot->Base.CollisionsInternal = newObj;
  bid = varargin_2.TreeInternal.Gravity[0];
  g_idx_1 = varargin_2.TreeInternal.Gravity[1];
  g_idx_2 = varargin_2.TreeInternal.Gravity[2];
  newrobot->Gravity[0] = bid;
  newrobot->Gravity[1] = g_idx_1;
  newrobot->Gravity[2] = g_idx_2;
  if (varargin_2.TreeInternal.NumBodies >= 1.0) {
    body = varargin_2.TreeInternal.Bodies[0];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
    if (b_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(b_obj.Length);
    }
    obj_size[0] = 1;
    obj_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &obj_data[0]);
    }
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[1], obj->_pobj0[0],
                      obj->_pobj1[0]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 2.0) {
    body = varargin_2.TreeInternal.Bodies[1];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
    if (b_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(b_obj.Length);
    }
    obj_size[0] = 1;
    obj_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &obj_data[0]);
    }
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[3], obj->_pobj0[2],
                      obj->_pobj1[1]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 3.0) {
    body = varargin_2.TreeInternal.Bodies[2];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
    if (b_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(b_obj.Length);
    }
    obj_size[0] = 1;
    obj_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &obj_data[0]);
    }
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[5], obj->_pobj0[4],
                      obj->_pobj1[2]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 4.0) {
    body = varargin_2.TreeInternal.Bodies[3];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
    if (b_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(b_obj.Length);
    }
    obj_size[0] = 1;
    obj_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &obj_data[0]);
    }
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[7], obj->_pobj0[6],
                      obj->_pobj1[3]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 5.0) {
    body = varargin_2.TreeInternal.Bodies[4];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
    if (b_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(b_obj.Length);
    }
    obj_size[0] = 1;
    obj_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &obj_data[0]);
    }
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[9], obj->_pobj0[8],
                      obj->_pobj1[4]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 6.0) {
    body = varargin_2.TreeInternal.Bodies[5];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
    if (b_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(b_obj.Length);
    }
    obj_size[0] = 1;
    obj_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &obj_data[0]);
    }
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[11],
                      obj->_pobj0[10], obj->_pobj1[5]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 7.0) {
    body = varargin_2.TreeInternal.Bodies[6];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
    if (b_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(b_obj.Length);
    }
    obj_size[0] = 1;
    obj_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &obj_data[0]);
    }
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[13],
                      obj->_pobj0[12], obj->_pobj1[6]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 8.0) {
    body = varargin_2.TreeInternal.Bodies[7];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
    if (b_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(b_obj.Length);
    }
    obj_size[0] = 1;
    obj_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &obj_data[0]);
    }
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[15],
                      obj->_pobj0[14], obj->_pobj1[7]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 9.0) {
    body = varargin_2.TreeInternal.Bodies[8];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
    if (b_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(b_obj.Length);
    }
    obj_size[0] = 1;
    obj_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &obj_data[0]);
    }
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[17],
                      obj->_pobj0[16], obj->_pobj1[8]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 10.0) {
    body = varargin_2.TreeInternal.Bodies[9];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
    if (b_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(b_obj.Length);
    }
    obj_size[0] = 1;
    obj_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &obj_data[0]);
    }
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[19],
                      obj->_pobj0[18], obj->_pobj1[9]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 11.0) {
    body = varargin_2.TreeInternal.Bodies[10];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
    if (b_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(b_obj.Length);
    }
    obj_size[0] = 1;
    obj_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &obj_data[0]);
    }
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[21],
                      obj->_pobj0[20], obj->_pobj1[10]);
  }
  obj->Tree = newrobot;
  obj->RigidBodyTreeHasBeenSet = true;
  obj->_pobj4.MaxNumIteration = 1500.0;
  obj->_pobj4.MaxTime = 10.0;
  obj->_pobj4.SolutionTolerance = 1.0E-6;
  obj->_pobj4.ConstraintsOn = true;
  obj->_pobj4.RandomRestart = true;
  obj->_pobj4.StepTolerance = 1.0E-12;
  obj->_pobj4.GradientTolerance = 5.0E-9;
  obj->_pobj4.ErrorChangeTolerance = 1.0E-12;
  obj->_pobj4.DampingBias = 0.0025;
  obj->_pobj4.UseErrorDamping = true;
  for (loop_ub = 0; loop_ub < 18; loop_ub++) {
    obj->_pobj4.Name[loop_ub] = cv13[loop_ub];
  }
  obj->_pobj4.TimeObj.StartTime.tv_sec = 0.0;
  obj->_pobj4.TimeObj.StartTime.tv_nsec = 0.0;
  obj->_pobj4.TimeObjInternal.StartTime.tv_sec = 0.0;
  obj->_pobj4.TimeObjInternal.StartTime.tv_nsec = 0.0;
  obj->_pobj4.matlabCodegenIsDeleted = false;
  obj->Solver = &obj->_pobj4;
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void generalizedInverseKinematics::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
    if (isInitialized == 1) {
      isInitialized = 2;
    }
  }
}

//
// Arguments    : double solverparams_MaxIterations
//                double solverparams_MaxTime
//                double solverparams_GradientTolerance
//                double solverparams_SolutionTolerance
//                boolean_T solverparams_EnforceJointLimits
//                boolean_T solverparams_AllowRandomRestart
//                double solverparams_StepTolerance
//                double solverparams_ErrorChangeTolerance
//                double solverparams_DampingBias
//                boolean_T solverparams_UseErrorDamping
// Return Type  : void
//
void generalizedInverseKinematics::set_SolverParameters(
    double solverparams_MaxIterations, double solverparams_MaxTime,
    double solverparams_GradientTolerance,
    double solverparams_SolutionTolerance,
    boolean_T solverparams_EnforceJointLimits,
    boolean_T solverparams_AllowRandomRestart,
    double solverparams_StepTolerance, double solverparams_ErrorChangeTolerance,
    double solverparams_DampingBias, boolean_T solverparams_UseErrorDamping)
{
  robotics::core::internal::ErrorDampedLevenbergMarquardt *obj;
  double b_expl_temp;
  double c_expl_temp;
  double d_expl_temp;
  double g_expl_temp;
  double params_DampingBias;
  double params_ErrorChangeTolerance;
  int ret;
  char expl_temp[18];
  boolean_T e_expl_temp;
  boolean_T f_expl_temp;
  boolean_T params_UseErrorDamping;
  Solver->getSolverParams(expl_temp, b_expl_temp, c_expl_temp, d_expl_temp,
                          e_expl_temp, f_expl_temp, g_expl_temp,
                          params_ErrorChangeTolerance, params_DampingBias,
                          params_UseErrorDamping);
  EnforceJointLimits = solverparams_EnforceJointLimits;
  for (ret = 0; ret < 18; ret++) {
    expl_temp[ret] = Solver->Name[ret];
  }
  ret = std::memcmp(&cv13[0], &expl_temp[0], 18);
  if (ret == 0) {
    ret = 1;
  } else {
    ret = -1;
  }
  switch (ret) {
  case 0:
    break;
  case 1:
    params_ErrorChangeTolerance = solverparams_ErrorChangeTolerance;
    params_DampingBias = solverparams_DampingBias;
    params_UseErrorDamping = solverparams_UseErrorDamping;
    break;
  }
  obj = Solver;
  obj->MaxNumIteration = solverparams_MaxIterations;
  obj->MaxTime = solverparams_MaxTime;
  obj->GradientTolerance = solverparams_GradientTolerance;
  obj->SolutionTolerance = solverparams_SolutionTolerance;
  obj->ConstraintsOn = true;
  obj->RandomRestart = solverparams_AllowRandomRestart;
  obj->StepTolerance = solverparams_StepTolerance;
  obj->ErrorChangeTolerance = params_ErrorChangeTolerance;
  obj->DampingBias = params_DampingBias;
  obj->UseErrorDamping = params_UseErrorDamping;
}

//
// Arguments    : const double varargin_1[9]
//                const constraintPoseTarget &varargin_2
//                const constraintJointBounds &varargin_3
//                const constraintDistanceBounds *varargin_4
//                const constraintDistanceBounds *varargin_5
//                const constraintDistanceBounds *varargin_6
//                const constraintDistanceBounds *varargin_7
//                const constraintDistanceBounds *varargin_8
//                const constraintDistanceBounds *varargin_9
//                const constraintDistanceBounds *varargin_10
//                const constraintDistanceBounds *varargin_11
//                const constraintDistanceBounds *varargin_12
//                const constraintDistanceBounds *varargin_13
//                const constraintDistanceBounds *varargin_14
//                const constraintDistanceBounds *varargin_15
//                const constraintDistanceBounds *varargin_16
//                const constraintDistanceBounds *varargin_17
//                const constraintDistanceBounds *varargin_18
//                const constraintDistanceBounds *varargin_19
//                const constraintDistanceBounds *varargin_20
//                const constraintDistanceBounds *varargin_21
//                const constraintDistanceBounds *varargin_22
//                const constraintDistanceBounds *varargin_23
//                double varargout_1[9]
//                struct0_T *varargout_2
// Return Type  : void
//
void generalizedInverseKinematics::step(
    const double varargin_1[9], const constraintPoseTarget &varargin_2,
    const constraintJointBounds &varargin_3,
    const constraintDistanceBounds *varargin_4,
    const constraintDistanceBounds *varargin_5,
    const constraintDistanceBounds *varargin_6,
    const constraintDistanceBounds *varargin_7,
    const constraintDistanceBounds *varargin_8,
    const constraintDistanceBounds *varargin_9,
    const constraintDistanceBounds *varargin_10,
    const constraintDistanceBounds *varargin_11,
    const constraintDistanceBounds *varargin_12,
    const constraintDistanceBounds *varargin_13,
    const constraintDistanceBounds *varargin_14,
    const constraintDistanceBounds *varargin_15,
    const constraintDistanceBounds *varargin_16,
    const constraintDistanceBounds *varargin_17,
    const constraintDistanceBounds *varargin_18,
    const constraintDistanceBounds *varargin_19,
    const constraintDistanceBounds *varargin_20,
    const constraintDistanceBounds *varargin_21,
    const constraintDistanceBounds *varargin_22,
    const constraintDistanceBounds *varargin_23, double varargout_1[9],
    struct0_T *varargout_2)
{
  static const char b_cv[14]{'b', 'e', 's', 't', ' ', 'a', 'v',
                             'a', 'i', 'l', 'a', 'b', 'l', 'e'};
  static const char b_cv1[7]{'s', 'u', 'c', 'c', 'e', 's', 's'};
  generalizedInverseKinematics *obj;
  robotics::core::internal::ErrorDampedLevenbergMarquardt *e_obj;
  robotics::manip::internal::JointPositionBounds *c_obj;
  robotics::manip::internal::PoseTarget *b_obj;
  robotics::manip::internal::RigidBodyTree *f_obj;
  array<double, 2U> b_value;
  array<double, 2U> c_value;
  array<double, 2U> positionIndices;
  array<double, 2U> r;
  array<double, 1U> d_obj;
  array<double, 1U> maxval;
  array<double, 1U> newseed;
  array<double, 1U> xSolPrev;
  array<int, 2U> r1;
  double positionMap_data[528];
  double kinematicPath_data[264];
  double d_expl_temp;
  double e_expl_temp;
  double err;
  double errPrev;
  double iter;
  double iterations;
  double ndbl;
  double rrAttempts;
  double tol;
  int a_size[2];
  int i;
  int k;
  int kstr;
  int nm1d2;
  int trueCount;
  short tmp_data[264];
  char a_data[200];
  boolean_T b_expl_temp;
  boolean_T c_expl_temp;
  boolean_T exitg2;
  boolean_T f_expl_temp;
  robotics::core::internal::NLPSolverExitFlags exitFlag;
  robotics::core::internal::NLPSolverExitFlags exitFlagPrev;
  if (isInitialized != 1) {
    char expl_temp[18];
    isSetupComplete = false;
    isInitialized = 1;
    Problem.init(Tree);
    Solver->getSolverParams(expl_temp, iter, tol, errPrev, b_expl_temp,
                            c_expl_temp, d_expl_temp, e_expl_temp, ndbl,
                            f_expl_temp);
    b_expl_temp = EnforceJointLimits;
    Problem.set_EnforceJointLimits(b_expl_temp);
    Solver->ExtraArgs = &Problem;
    isSetupComplete = true;
  }
  obj = this;
  ::std::copy(&varargin_1[0], &varargin_1[9], &varargout_1[0]);
  Tree->validateConfigurationWithLimits(varargout_1);
  b_obj = Problem.Constraints.f1;
  b_obj->Weights.set_size(1, 2);
  b_obj->Weights[0] = varargin_2.Weights[0];
  b_obj->Weights[1] = varargin_2.Weights[1];
  b_obj->get_EndEffector(a_data, a_size);
  b_expl_temp = false;
  if (a_size[1] == 17) {
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 17) {
        if (a_data[kstr] != varargin_2.EndEffector[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_expl_temp = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_expl_temp) {
    b_obj->EndEffectorIndex =
        b_obj->Tree->validateInputBodyName(varargin_2.EndEffector);
  }
  b_obj->get_ReferenceBody(a_data, a_size);
  if (static_cast<unsigned char>(a_size[1]) != 0) {
    b_obj->ReferenceBodyIndex = 0.0;
  }
  for (i = 0; i < 16; i++) {
    b_obj->TargetTransform[i] = varargin_2.TargetTransform[i];
  }
  b_obj->BoundsInternal[b_obj->BoundsInternal.size(0)] =
      varargin_2.OrientationTolerance;
  b_obj->BoundsInternal[b_obj->BoundsInternal.size(0) + 1] =
      varargin_2.PositionTolerance;
  c_obj = Problem.Constraints.f2;
  kstr = varargin_3.WeightsInternal.size(1);
  b_value.set_size(1, kstr);
  for (i = 0; i < kstr; i++) {
    b_value[i] = varargin_3.WeightsInternal[i];
  }
  c_obj->Weights.set_size(1, kstr);
  for (i = 0; i < kstr; i++) {
    c_obj->Weights[i] = b_value[i];
  }
  i = varargin_3.BoundsInternal.size(0);
  c_value.set_size(i, 2);
  kstr = varargin_3.BoundsInternal.size(0) << 1;
  for (k = 0; k < kstr; k++) {
    c_value[k] = varargin_3.BoundsInternal[k];
  }
  c_obj->BoundsInternal.set_size(i, 2);
  for (i = 0; i < kstr; i++) {
    c_obj->BoundsInternal[i] = c_value[i];
  }
  Problem.Constraints.f3->update(varargin_4);
  Problem.Constraints.f4->update(varargin_5);
  Problem.Constraints.f5->update(varargin_6);
  Problem.Constraints.f6->update(varargin_7);
  Problem.Constraints.f7->update(varargin_8);
  Problem.Constraints.f8->update(varargin_9);
  Problem.Constraints.f9->update(varargin_10);
  Problem.Constraints.f10->update(varargin_11);
  Problem.Constraints.f11->update(varargin_12);
  Problem.Constraints.f12->update(varargin_13);
  Problem.Constraints.f13->update(varargin_14);
  Problem.Constraints.f14->update(varargin_15);
  Problem.Constraints.f15->update(varargin_16);
  Problem.Constraints.f16->update(varargin_17);
  Problem.Constraints.f17->update(varargin_18);
  Problem.Constraints.f18->update(varargin_19);
  Problem.Constraints.f19->update(varargin_20);
  Problem.Constraints.f20->update(varargin_21);
  Problem.Constraints.f21->update(varargin_22);
  Problem.Constraints.f22->update(varargin_23);
  Problem.updateDesignVariableBounds();
  kstr = static_cast<int>(Problem.NumVariables);
  Problem.LastX.set_size(kstr);
  for (i = 0; i < kstr; i++) {
    Problem.LastX[i] = 0.0;
  }
  d_obj.set_size(Problem.LastX.size(0));
  kstr = Problem.LastX.size(0) - 1;
  for (i = 0; i <= kstr; i++) {
    d_obj[i] = Problem.LastX[i];
  }
  Problem.residualsInternal(d_obj, newseed, r);
  kstr = newseed.size(0);
  Problem.LastF.set_size(newseed.size(0));
  for (i = 0; i < kstr; i++) {
    Problem.LastF[i] = newseed[i];
  }
  Problem.LastJ.set_size(r.size(0), r.size(1));
  kstr = r.size(0) * r.size(1);
  for (i = 0; i < kstr; i++) {
    Problem.LastJ[i] = r[i];
  }
  Problem.set_EnforceJointLimits(EnforceJointLimits);
  c_value.set_size(Problem.DesignVariableBoundsInternal.size(0), 2);
  kstr = Problem.DesignVariableBoundsInternal.size(0) << 1;
  for (i = 0; i < kstr; i++) {
    c_value[i] = Problem.DesignVariableBoundsInternal[i];
  }
  err = Problem.NumPositions + 1.0;
  if (err > c_value.size(0)) {
    i = 0;
    k = -1;
  } else {
    i = static_cast<int>(err) - 1;
    k = c_value.size(0) - 1;
  }
  kstr = static_cast<int>(Problem.NumSlacks);
  d_obj.set_size(kstr + 9);
  for (nm1d2 = 0; nm1d2 < 9; nm1d2++) {
    d_obj[nm1d2] = varargout_1[nm1d2];
  }
  for (nm1d2 = 0; nm1d2 < kstr; nm1d2++) {
    d_obj[nm1d2 + 9] = 0.0;
  }
  Problem.residuals(d_obj, newseed);
  kstr = k - i;
  if (kstr + 1 == newseed.size(0)) {
    maxval.set_size(kstr + 1);
    for (k = 0; k <= kstr; k++) {
      err = c_value[i + k];
      iter = newseed[k];
      maxval[k] = std::fmax(err, iter);
    }
  } else {
    d_obj.set_size(kstr + 1);
    for (k = 0; k <= kstr; k++) {
      d_obj[k] = c_value[i + k];
    }
    internal::expand_max(d_obj, newseed, maxval);
  }
  if (kstr + 1 == maxval.size(0)) {
    newseed.set_size(kstr + 1);
    for (k = 0; k <= kstr; k++) {
      err = c_value[(i + k) + c_value.size(0)];
      iter = maxval[k];
      newseed[k] = std::fmin(err, iter);
    }
  } else {
    d_obj.set_size(kstr + 1);
    for (k = 0; k <= kstr; k++) {
      d_obj[k] = c_value[(i + k) + c_value.size(0)];
    }
    internal::expand_min(d_obj, maxval, newseed);
  }
  e_obj = Solver;
  e_obj->MaxNumIterationInternal = e_obj->MaxNumIteration;
  e_obj->MaxTimeInternal = e_obj->MaxTime;
  e_obj->SeedInternal.set_size(newseed.size(0) + 9);
  for (i = 0; i < 9; i++) {
    e_obj->SeedInternal[i] = varargout_1[i];
  }
  kstr = newseed.size(0);
  for (i = 0; i < kstr; i++) {
    e_obj->SeedInternal[i + 9] = newseed[i];
  }
  tol = e_obj->SolutionTolerance;
  e_obj->TimeObj.StartTime.tv_sec = tic(e_obj->TimeObj.StartTime.tv_nsec);
  exitFlag = e_obj->solveInternal(xSolPrev, err, iter);
  rrAttempts = 0.0;
  iterations = iter;
  errPrev = err;
  exitFlagPrev = exitFlag;
  exitg2 = false;
  while ((!exitg2) && (e_obj->RandomRestart && (err > tol))) {
    e_obj->MaxNumIterationInternal -= iter;
    err =
        toc(e_obj->TimeObj.StartTime.tv_sec, e_obj->TimeObj.StartTime.tv_nsec);
    e_obj->MaxTimeInternal = e_obj->MaxTime - err;
    if (e_obj->MaxNumIterationInternal <= 0.0) {
      exitFlag =
          robotics::core::internal::NLPSolverExitFlags::IterationLimitExceeded;
    }
    if ((exitFlag == robotics::core::internal::NLPSolverExitFlags::
                         IterationLimitExceeded) ||
        (exitFlag ==
         robotics::core::internal::NLPSolverExitFlags::TimeLimitExceeded)) {
      exitFlagPrev = exitFlag;
      exitg2 = true;
    } else {
      rigidBodyJoint::randomConfig(e_obj->ExtraArgs, newseed);
      kstr = newseed.size(0);
      e_obj->SeedInternal.set_size(newseed.size(0));
      for (i = 0; i < kstr; i++) {
        e_obj->SeedInternal[i] = newseed[i];
      }
      exitFlag = e_obj->solveInternal(maxval, err, iter);
      if (err < errPrev) {
        kstr = maxval.size(0);
        xSolPrev.set_size(maxval.size(0));
        for (i = 0; i < kstr; i++) {
          xSolPrev[i] = maxval[i];
        }
        errPrev = err;
        exitFlagPrev = exitFlag;
      }
      rrAttempts++;
      iterations += iter;
    }
  }
  if (errPrev < tol) {
    varargout_2->Status.size[0] = 1;
    varargout_2->Status.size[1] = 7;
    for (i = 0; i < 7; i++) {
      varargout_2->Status.data[i] = b_cv1[i];
    }
  } else {
    varargout_2->Status.size[0] = 1;
    varargout_2->Status.size[1] = 14;
    for (i = 0; i < 14; i++) {
      varargout_2->Status.data[i] = b_cv[i];
    }
  }
  err = obj->Tree->PositionNumber;
  if (err < 1.0) {
    kstr = 0;
  } else {
    kstr = static_cast<int>(err);
  }
  obj->Problem.get_KinematicPath(kinematicPath_data, a_size);
  d_obj.set_size(kstr);
  for (i = 0; i < kstr; i++) {
    d_obj[i] = xSolPrev[i];
  }
  obj->Problem.constraintViolations(d_obj, varargout_2->ConstraintViolations);
  f_obj = obj->Tree;
  kstr = a_size[1];
  trueCount = 0;
  nm1d2 = 0;
  for (int b_i{0}; b_i < kstr; b_i++) {
    if (kinematicPath_data[b_i] != 0.0) {
      trueCount++;
      tmp_data[nm1d2] = static_cast<short>(b_i);
      nm1d2++;
    }
  }
  for (i = 0; i < 2; i++) {
    for (k = 0; k < trueCount; k++) {
      positionMap_data[k + trueCount * i] =
          f_obj->PositionDoFMap
              [(static_cast<int>(kinematicPath_data[tmp_data[k]]) + 11 * i) -
               1];
    }
  }
  positionIndices.set_size(1, static_cast<int>(f_obj->PositionNumber));
  kstr = static_cast<int>(f_obj->PositionNumber);
  for (i = 0; i < kstr; i++) {
    positionIndices[i] = 0.0;
  }
  err = 0.0;
  for (int b_i{0}; b_i < trueCount; b_i++) {
    iter = positionMap_data[b_i + trueCount];
    d_expl_temp = iter - positionMap_data[b_i];
    if (d_expl_temp + 1.0 > 0.0) {
      if (d_expl_temp + 1.0 < 1.0) {
        b_value.set_size(1, 0);
      } else {
        b_value.set_size(1, static_cast<int>((d_expl_temp + 1.0) - 1.0) + 1);
        kstr = static_cast<int>((d_expl_temp + 1.0) - 1.0);
        for (i = 0; i <= kstr; i++) {
          b_value[i] = static_cast<double>(i) + 1.0;
        }
      }
      kstr = b_value.size(1);
      r1.set_size(1, b_value.size(1));
      for (i = 0; i < kstr; i++) {
        r1[i] = static_cast<int>(err + b_value[i]);
      }
      e_expl_temp = positionMap_data[b_i];
      if (std::isnan(e_expl_temp) || std::isnan(iter)) {
        b_value.set_size(1, 1);
        b_value[0] = rtNaN;
      } else if (iter < e_expl_temp) {
        b_value.set_size(1, 0);
      } else if ((std::isinf(e_expl_temp) || std::isinf(iter)) &&
                 (e_expl_temp == iter)) {
        b_value.set_size(1, 1);
        b_value[0] = rtNaN;
      } else if (std::floor(e_expl_temp) == e_expl_temp) {
        kstr = static_cast<int>(d_expl_temp);
        b_value.set_size(1, static_cast<int>(d_expl_temp) + 1);
        for (i = 0; i <= kstr; i++) {
          b_value[i] = e_expl_temp + static_cast<double>(i);
        }
      } else {
        ndbl = std::floor(d_expl_temp + 0.5);
        tol = e_expl_temp + ndbl;
        errPrev = tol - iter;
        if (std::abs(errPrev) <
            4.4408920985006262E-16 *
                std::fmax(std::abs(e_expl_temp), std::abs(iter))) {
          ndbl++;
          tol = iter;
        } else if (errPrev > 0.0) {
          tol = e_expl_temp + (ndbl - 1.0);
        } else {
          ndbl++;
        }
        if (ndbl >= 0.0) {
          kstr = static_cast<int>(ndbl);
        } else {
          kstr = 0;
        }
        b_value.set_size(1, kstr);
        if (kstr > 0) {
          b_value[0] = e_expl_temp;
          if (kstr > 1) {
            b_value[kstr - 1] = tol;
            nm1d2 = (kstr - 1) / 2;
            for (k = 0; k <= nm1d2 - 2; k++) {
              b_value[k + 1] = e_expl_temp + (static_cast<double>(k) + 1.0);
              b_value[(kstr - k) - 2] = tol - (static_cast<double>(k) + 1.0);
            }
            if (nm1d2 << 1 == kstr - 1) {
              b_value[nm1d2] = (positionMap_data[b_i] + tol) / 2.0;
            } else {
              b_value[nm1d2] =
                  positionMap_data[b_i] + static_cast<double>(nm1d2);
              b_value[nm1d2 + 1] = tol - static_cast<double>(nm1d2);
            }
          }
        }
      }
      kstr = b_value.size(1) - 1;
      for (i = 0; i <= kstr; i++) {
        positionIndices[r1[i] - 1] = b_value[i];
      }
      err += d_expl_temp + 1.0;
    }
  }
  if (err < 1.0) {
    kstr = 0;
  } else {
    kstr = static_cast<int>(err);
  }
  positionIndices.set_size(positionIndices.size(0), kstr);
  for (i = 0; i < kstr; i++) {
    err = positionIndices[i];
    varargout_1[static_cast<int>(err) - 1] =
        xSolPrev[static_cast<int>(err) - 1];
  }
  varargout_2->Iterations = iterations;
  varargout_2->NumRandomRestarts = rrAttempts;
  varargout_2->ExitFlag = static_cast<double>(exitFlagPrev);
}

} // namespace coder

//
// File trailer for generalizedInverseKinematics.cpp
//
// [EOF]
//
