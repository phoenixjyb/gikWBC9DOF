//
// File: generalizedInverseKinematics.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "generalizedInverseKinematics.h"
#include "CharacterVector.h"
#include "CollisionGeometry.h"
#include "CollisionSet.h"
#include "ErrorDampedLevenbergMarquardt.h"
#include "GIKProblem.h"
#include "GIKSolver.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "SystemTimeProvider.h"
#include "constraintDistanceBounds.h"
#include "constraintJointBounds.h"
#include "constraintPoseTarget.h"
#include "find.h"
#include "ixfun.h"
#include "rigidBodyJoint.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"
#include "solveGIKStepWrapper_data.h"
#include "solveGIKStepWrapper_types.h"
#include "solveGIKStepWrapper_types1.h"
#include "string1.h"
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
namespace gik9dof {
static const char cv13[18]{'L', 'e', 'v', 'e', 'n', 'b', 'e', 'r', 'g',
                           'M', 'a', 'r', 'q', 'u', 'a', 'r', 'd', 't'};

}

// Function Definitions
//
// Arguments    : void
// Return Type  : generalizedInverseKinematics
//
namespace gik9dof {
namespace coder {
generalizedInverseKinematics::generalizedInverseKinematics() = default;

//
// Arguments    : void
// Return Type  : void
//
generalizedInverseKinematics::~generalizedInverseKinematics() = default;

//
// Arguments    : GIKSolver *aInstancePtr
//                rigidBodyTree &varargin_2
// Return Type  : generalizedInverseKinematics *
//
generalizedInverseKinematics *
generalizedInverseKinematics::init(GIKSolver *aInstancePtr,
                                   rigidBodyTree &varargin_2)
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
  robotics::manip::internal::b_RigidBodyTree *newrobot;
  ::coder::array<char, 2U> b_obj_data;
  double bid;
  int obj_size[2];
  int loop_ub;
  char obj_data[200];
  obj = this;
  obj->EnforceJointLimits = true;
  obj->isInitialized = 0;
  newrobot = obj->_pobj3.init(aInstancePtr);
  b_obj = varargin_2.TreeInternal.Base.NameInternal;
  if (b_obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(b_obj.Length);
  }
  if (loop_ub - 1 >= 0) {
    ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &obj_data[0]);
  }
  b_obj_data.set(&obj_data[0], 1, loop_ub);
  bid = newrobot->findBodyIndexByName(b_obj_data);
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
  newObj =
      (&(&(&(&(&obj->_pobj2[0])[0])[0])[0])[0])[0].init(d_obj->MaxElements);
  newObj->Size = d_obj->Size;
  newObj->Tags = d_obj->Tags;
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
    newrobot->addBody(body, obj_data, obj_size,
                      (&(&(&(&(&obj->_pobj2[0])[0])[0])[0])[0])[1],
                      (&(&(&(&(&obj->_pobj0[0])[0])[0])[0])[0])[0],
                      (&(&(&(&(&obj->_pobj1[0])[0])[0])[0])[0])[0]);
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
    newrobot->addBody(body, obj_data, obj_size,
                      (&(&(&(&(&obj->_pobj2[0])[0])[0])[0])[0])[3],
                      (&(&(&(&(&obj->_pobj0[0])[0])[0])[0])[0])[2],
                      (&(&(&(&(&obj->_pobj1[0])[0])[0])[0])[0])[1]);
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
    newrobot->addBody(body, obj_data, obj_size,
                      (&(&(&(&(&obj->_pobj2[0])[0])[0])[0])[0])[5],
                      (&(&(&(&(&obj->_pobj0[0])[0])[0])[0])[0])[4],
                      (&(&(&(&(&obj->_pobj1[0])[0])[0])[0])[0])[2]);
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
    newrobot->addBody(body, obj_data, obj_size,
                      (&(&(&(&(&obj->_pobj2[0])[0])[0])[0])[0])[7],
                      (&(&(&(&(&obj->_pobj0[0])[0])[0])[0])[0])[6],
                      (&(&(&(&(&obj->_pobj1[0])[0])[0])[0])[0])[3]);
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
    newrobot->addBody(body, obj_data, obj_size,
                      (&(&(&(&(&obj->_pobj2[0])[0])[0])[0])[0])[9],
                      (&(&(&(&(&obj->_pobj0[0])[0])[0])[0])[0])[8],
                      (&(&(&(&(&obj->_pobj1[0])[0])[0])[0])[0])[4]);
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
    newrobot->addBody(body, obj_data, obj_size,
                      (&(&(&(&(&obj->_pobj2[0])[0])[0])[0])[0])[11],
                      (&(&(&(&(&obj->_pobj0[0])[0])[0])[0])[0])[10],
                      (&(&(&(&(&obj->_pobj1[0])[0])[0])[0])[0])[5]);
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
    newrobot->addBody(body, obj_data, obj_size,
                      (&(&(&(&(&obj->_pobj2[0])[0])[0])[0])[0])[13],
                      (&(&(&(&(&obj->_pobj0[0])[0])[0])[0])[0])[12],
                      (&(&(&(&(&obj->_pobj1[0])[0])[0])[0])[0])[6]);
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
    newrobot->addBody(body, obj_data, obj_size,
                      (&(&(&(&(&obj->_pobj2[0])[0])[0])[0])[0])[15],
                      (&(&(&(&(&obj->_pobj0[0])[0])[0])[0])[0])[14],
                      (&(&(&(&(&obj->_pobj1[0])[0])[0])[0])[0])[7]);
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
    newrobot->addBody(body, obj_data, obj_size,
                      (&(&(&(&(&obj->_pobj2[0])[0])[0])[0])[0])[17],
                      (&(&(&(&(&obj->_pobj0[0])[0])[0])[0])[0])[16],
                      (&(&(&(&(&obj->_pobj1[0])[0])[0])[0])[0])[8]);
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
    newrobot->addBody(body, obj_data, obj_size,
                      (&(&(&(&(&obj->_pobj2[0])[0])[0])[0])[0])[19],
                      (&(&(&(&(&obj->_pobj0[0])[0])[0])[0])[0])[18],
                      (&(&(&(&(&obj->_pobj1[0])[0])[0])[0])[0])[9]);
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
    newrobot->addBody(body, obj_data, obj_size,
                      (&(&(&(&(&obj->_pobj2[0])[0])[0])[0])[0])[21],
                      (&(&(&(&(&obj->_pobj0[0])[0])[0])[0])[0])[20],
                      (&(&(&(&(&obj->_pobj1[0])[0])[0])[0])[0])[10]);
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
// Arguments    : GIKSolver *aInstancePtr
//                double varargin_1[9]
//                const constraintPoseTarget &varargin_2
//                const constraintJointBounds &varargin_3
//                constraintDistanceBounds *varargin_4
//                constraintDistanceBounds *varargin_5
//                constraintDistanceBounds *varargin_6
//                constraintDistanceBounds *varargin_7
//                constraintDistanceBounds *varargin_8
//                constraintDistanceBounds *varargin_9
//                constraintDistanceBounds *varargin_10
//                constraintDistanceBounds *varargin_11
//                constraintDistanceBounds *varargin_12
//                constraintDistanceBounds *varargin_13
//                constraintDistanceBounds *varargin_14
//                constraintDistanceBounds *varargin_15
//                constraintDistanceBounds *varargin_16
//                constraintDistanceBounds *varargin_17
//                constraintDistanceBounds *varargin_18
//                constraintDistanceBounds *varargin_19
//                constraintDistanceBounds *varargin_20
//                constraintDistanceBounds *varargin_21
//                constraintDistanceBounds *varargin_22
//                constraintDistanceBounds *varargin_23
//                struct0_T *varargout_2
// Return Type  : void
//
void generalizedInverseKinematics::step(
    GIKSolver *aInstancePtr, double varargin_1[9],
    const constraintPoseTarget &varargin_2,
    const constraintJointBounds &varargin_3,
    constraintDistanceBounds *varargin_4, constraintDistanceBounds *varargin_5,
    constraintDistanceBounds *varargin_6, constraintDistanceBounds *varargin_7,
    constraintDistanceBounds *varargin_8, constraintDistanceBounds *varargin_9,
    constraintDistanceBounds *varargin_10,
    constraintDistanceBounds *varargin_11,
    constraintDistanceBounds *varargin_12,
    constraintDistanceBounds *varargin_13,
    constraintDistanceBounds *varargin_14,
    constraintDistanceBounds *varargin_15,
    constraintDistanceBounds *varargin_16,
    constraintDistanceBounds *varargin_17,
    constraintDistanceBounds *varargin_18,
    constraintDistanceBounds *varargin_19,
    constraintDistanceBounds *varargin_20,
    constraintDistanceBounds *varargin_21,
    constraintDistanceBounds *varargin_22,
    constraintDistanceBounds *varargin_23, struct0_T *varargout_2)
{
  static const char b_cv[14]{'b', 'e', 's', 't', ' ', 'a', 'v',
                             'a', 'i', 'l', 'a', 'b', 'l', 'e'};
  static const char b_cv1[7]{'s', 'u', 'c', 'c', 'e', 's', 's'};
  robotics::core::internal::ErrorDampedLevenbergMarquardt *b_obj;
  robotics::manip::internal::GIKProblem *problem;
  robotics::manip::internal::b_RigidBodyTree *obj;
  ::coder::array<double, 2U> limits;
  ::coder::array<double, 2U> positionIndices;
  ::coder::array<double, 2U> y;
  ::coder::array<double, 1U> b_limits;
  ::coder::array<double, 1U> b_varargin_1;
  ::coder::array<double, 1U> s;
  ::coder::array<double, 1U> xSolPrev;
  ::coder::array<int, 2U> r;
  double positionMap_data[528];
  double kinematicPath_data[264];
  double qr_data[77];
  double apnd;
  double cdiff;
  double err;
  double errPrev;
  double iter;
  double iterations;
  double ndbl;
  double rrAttempts;
  double tol;
  int indicesUpperBoundViolation_data[9];
  int kinematicPath_size[2];
  int i;
  int indicesUpperBoundViolation_size;
  int k;
  int nm1d2;
  int trueCount;
  short tmp_data[264];
  boolean_T lbOK[9];
  boolean_T ubOK[9];
  boolean_T b_expl_temp;
  boolean_T c_expl_temp;
  boolean_T d_expl_temp;
  boolean_T exitg1;
  boolean_T guard1;
  robotics::core::internal::NLPSolverExitFlags exitFlag;
  robotics::core::internal::NLPSolverExitFlags exitFlagPrev;
  if (isInitialized != 1) {
    char expl_temp[18];
    isSetupComplete = false;
    isInitialized = 1;
    Problem.init(Tree);
    Solver->getSolverParams(expl_temp, iter, tol, errPrev, b_expl_temp,
                            c_expl_temp, ndbl, apnd, cdiff, d_expl_temp);
    b_expl_temp = EnforceJointLimits;
    Problem.set_EnforceJointLimits(b_expl_temp);
    Solver->ExtraArgs = &Problem;
    isSetupComplete = true;
  }
  obj = Tree;
  obj->get_JointPositionLimits(limits);
  if (limits.size(0) == 9) {
    for (i = 0; i < 9; i++) {
      ubOK[i] = (varargin_1[i] <=
                 limits[i + limits.size(0)] + 4.4408920985006262E-16);
    }
  } else {
    binary_expand_op_30(ubOK, varargin_1, limits);
  }
  if (limits.size(0) == 9) {
    for (i = 0; i < 9; i++) {
      lbOK[i] = (varargin_1[i] >= limits[i] - 4.4408920985006262E-16);
    }
  } else {
    binary_expand_op_29(lbOK, varargin_1, limits);
  }
  b_expl_temp = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 8)) {
    if (!ubOK[k]) {
      b_expl_temp = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  guard1 = false;
  if (b_expl_temp) {
    b_expl_temp = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= 8)) {
      if (!lbOK[k]) {
        b_expl_temp = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
    if (!b_expl_temp) {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    for (i = 0; i < 9; i++) {
      ubOK[i] = !ubOK[i];
    }
    indicesUpperBoundViolation_size =
        eml_find(ubOK, indicesUpperBoundViolation_data);
    for (i = 0; i < indicesUpperBoundViolation_size; i++) {
      k = indicesUpperBoundViolation_data[i];
      varargin_1[k - 1] = limits[(k + limits.size(0)) - 1];
    }
    for (i = 0; i < 9; i++) {
      lbOK[i] = !lbOK[i];
    }
    indicesUpperBoundViolation_size =
        eml_find(lbOK, indicesUpperBoundViolation_data);
    for (i = 0; i < indicesUpperBoundViolation_size; i++) {
      k = indicesUpperBoundViolation_data[i];
      varargin_1[k - 1] = limits[k - 1];
    }
  }
  Problem.update(varargin_2, varargin_3, varargin_4, varargin_5, varargin_6,
                 varargin_7, varargin_8, varargin_9, varargin_10, varargin_11,
                 varargin_12, varargin_13, varargin_14, varargin_15,
                 varargin_16, varargin_17, varargin_18, varargin_19,
                 varargin_20, varargin_21, varargin_22, varargin_23);
  Problem.set_EnforceJointLimits(EnforceJointLimits);
  limits.set_size(Problem.DesignVariableBoundsInternal.size(0), 2);
  indicesUpperBoundViolation_size = Problem.DesignVariableBoundsInternal.size(0)
                                    << 1;
  for (i = 0; i < indicesUpperBoundViolation_size; i++) {
    limits[i] = Problem.DesignVariableBoundsInternal[i];
  }
  err = Problem.NumPositions + 1.0;
  if (err > limits.size(0)) {
    i = 0;
    k = -1;
  } else {
    i = static_cast<int>(err) - 1;
    k = limits.size(0) - 1;
  }
  indicesUpperBoundViolation_size = static_cast<int>(Problem.NumSlacks);
  b_varargin_1.set_size(indicesUpperBoundViolation_size + 9);
  for (trueCount = 0; trueCount < 9; trueCount++) {
    b_varargin_1[trueCount] = varargin_1[trueCount];
  }
  for (trueCount = 0; trueCount < indicesUpperBoundViolation_size;
       trueCount++) {
    b_varargin_1[trueCount + 9] = 0.0;
  }
  Problem.residuals(b_varargin_1, s);
  indicesUpperBoundViolation_size = k - i;
  if (indicesUpperBoundViolation_size + 1 == s.size(0)) {
    b_varargin_1.set_size(indicesUpperBoundViolation_size + 1);
    for (k = 0; k <= indicesUpperBoundViolation_size; k++) {
      err = limits[i + k];
      iter = s[k];
      b_varargin_1[k] = std::fmax(err, iter);
    }
  } else {
    b_limits.set_size(indicesUpperBoundViolation_size + 1);
    for (k = 0; k <= indicesUpperBoundViolation_size; k++) {
      b_limits[k] = limits[i + k];
    }
    internal::expand_max(b_limits, s, b_varargin_1);
  }
  if (indicesUpperBoundViolation_size + 1 == b_varargin_1.size(0)) {
    s.set_size(indicesUpperBoundViolation_size + 1);
    for (k = 0; k <= indicesUpperBoundViolation_size; k++) {
      err = limits[(i + k) + limits.size(0)];
      iter = b_varargin_1[k];
      s[k] = std::fmin(err, iter);
    }
  } else {
    b_limits.set_size(indicesUpperBoundViolation_size + 1);
    for (k = 0; k <= indicesUpperBoundViolation_size; k++) {
      b_limits[k] = limits[(i + k) + limits.size(0)];
    }
    internal::expand_min(b_limits, b_varargin_1, s);
  }
  b_obj = Solver;
  b_obj->MaxNumIterationInternal = b_obj->MaxNumIteration;
  b_obj->MaxTimeInternal = b_obj->MaxTime;
  b_obj->SeedInternal.set_size(s.size(0) + 9);
  for (i = 0; i < 9; i++) {
    b_obj->SeedInternal[i] = varargin_1[i];
  }
  indicesUpperBoundViolation_size = s.size(0);
  for (i = 0; i < indicesUpperBoundViolation_size; i++) {
    b_obj->SeedInternal[i + 9] = s[i];
  }
  tol = b_obj->SolutionTolerance;
  b_obj->TimeObj.StartTime.tv_sec =
      tic(aInstancePtr, b_obj->TimeObj.StartTime.tv_nsec);
  exitFlag = b_obj->solveInternal(aInstancePtr, xSolPrev, err, iter);
  rrAttempts = 0.0;
  iterations = iter;
  errPrev = err;
  exitFlagPrev = exitFlag;
  exitg1 = false;
  while ((!exitg1) && (b_obj->RandomRestart && (err > tol))) {
    b_obj->MaxNumIterationInternal -= iter;
    err = toc(aInstancePtr, b_obj->TimeObj.StartTime.tv_sec,
              b_obj->TimeObj.StartTime.tv_nsec);
    b_obj->MaxTimeInternal = b_obj->MaxTime - err;
    if (b_obj->MaxNumIterationInternal <= 0.0) {
      exitFlag =
          robotics::core::internal::NLPSolverExitFlags::IterationLimitExceeded;
    }
    if ((exitFlag == robotics::core::internal::NLPSolverExitFlags::
                         IterationLimitExceeded) ||
        (exitFlag ==
         robotics::core::internal::NLPSolverExitFlags::TimeLimitExceeded)) {
      exitFlagPrev = exitFlag;
      exitg1 = true;
    } else {
      problem = b_obj->ExtraArgs;
      nm1d2 = problem->Tree->randomJointPositions(aInstancePtr, qr_data);
      limits.set_size(problem->DesignVariableBoundsInternal.size(0), 2);
      indicesUpperBoundViolation_size =
          problem->DesignVariableBoundsInternal.size(0) << 1;
      for (i = 0; i < indicesUpperBoundViolation_size; i++) {
        limits[i] = problem->DesignVariableBoundsInternal[i];
      }
      err = problem->NumPositions + 1.0;
      if (err > limits.size(0)) {
        i = 0;
        k = -1;
      } else {
        i = static_cast<int>(err) - 1;
        k = limits.size(0) - 1;
      }
      indicesUpperBoundViolation_size = static_cast<int>(problem->NumSlacks);
      b_varargin_1.set_size(nm1d2 + indicesUpperBoundViolation_size);
      for (trueCount = 0; trueCount < nm1d2; trueCount++) {
        b_varargin_1[trueCount] = qr_data[trueCount];
      }
      for (trueCount = 0; trueCount < indicesUpperBoundViolation_size;
           trueCount++) {
        b_varargin_1[trueCount + nm1d2] = 0.0;
      }
      problem->residuals(b_varargin_1, s);
      indicesUpperBoundViolation_size = k - i;
      if (indicesUpperBoundViolation_size + 1 == s.size(0)) {
        b_varargin_1.set_size(indicesUpperBoundViolation_size + 1);
        for (k = 0; k <= indicesUpperBoundViolation_size; k++) {
          err = limits[i + k];
          iter = s[k];
          b_varargin_1[k] = std::fmax(err, iter);
        }
      } else {
        b_limits.set_size(indicesUpperBoundViolation_size + 1);
        for (k = 0; k <= indicesUpperBoundViolation_size; k++) {
          b_limits[k] = limits[i + k];
        }
        internal::expand_max(b_limits, s, b_varargin_1);
      }
      if (indicesUpperBoundViolation_size + 1 == b_varargin_1.size(0)) {
        s.set_size(indicesUpperBoundViolation_size + 1);
        for (k = 0; k <= indicesUpperBoundViolation_size; k++) {
          err = limits[(i + k) + limits.size(0)];
          iter = b_varargin_1[k];
          s[k] = std::fmin(err, iter);
        }
      } else {
        b_limits.set_size(indicesUpperBoundViolation_size + 1);
        for (k = 0; k <= indicesUpperBoundViolation_size; k++) {
          b_limits[k] = limits[(i + k) + limits.size(0)];
        }
        internal::expand_min(b_limits, b_varargin_1, s);
      }
      b_obj->SeedInternal.set_size(nm1d2 + s.size(0));
      for (i = 0; i < nm1d2; i++) {
        b_obj->SeedInternal[i] = qr_data[i];
      }
      indicesUpperBoundViolation_size = s.size(0);
      for (i = 0; i < indicesUpperBoundViolation_size; i++) {
        b_obj->SeedInternal[i + nm1d2] = s[i];
      }
      exitFlag = b_obj->solveInternal(aInstancePtr, s, err, iter);
      if (err < errPrev) {
        indicesUpperBoundViolation_size = s.size(0);
        xSolPrev.set_size(s.size(0));
        for (i = 0; i < indicesUpperBoundViolation_size; i++) {
          xSolPrev[i] = s[i];
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
  err = Tree->PositionNumber;
  if (err < 1.0) {
    indicesUpperBoundViolation_size = 0;
  } else {
    indicesUpperBoundViolation_size = static_cast<int>(err);
  }
  Problem.get_KinematicPath(kinematicPath_data, kinematicPath_size);
  b_varargin_1.set_size(indicesUpperBoundViolation_size);
  for (i = 0; i < indicesUpperBoundViolation_size; i++) {
    b_varargin_1[i] = xSolPrev[i];
  }
  Problem.constraintViolations(b_varargin_1, varargout_2->ConstraintViolations);
  obj = Tree;
  indicesUpperBoundViolation_size = kinematicPath_size[1] - 1;
  trueCount = 0;
  nm1d2 = 0;
  for (int b_i{0}; b_i <= indicesUpperBoundViolation_size; b_i++) {
    if (kinematicPath_data[b_i] != 0.0) {
      trueCount++;
      tmp_data[nm1d2] = static_cast<short>(b_i);
      nm1d2++;
    }
  }
  for (i = 0; i < 2; i++) {
    for (k = 0; k < trueCount; k++) {
      positionMap_data[k + trueCount * i] =
          obj->PositionDoFMap
              [(static_cast<int>(kinematicPath_data[tmp_data[k]]) + 11 * i) -
               1];
    }
  }
  positionIndices.set_size(1, static_cast<int>(obj->PositionNumber));
  indicesUpperBoundViolation_size = static_cast<int>(obj->PositionNumber);
  for (i = 0; i < indicesUpperBoundViolation_size; i++) {
    positionIndices[i] = 0.0;
  }
  err = 0.0;
  for (int b_i{0}; b_i < trueCount; b_i++) {
    iter = positionMap_data[b_i + trueCount];
    tol = iter - positionMap_data[b_i];
    if (tol + 1.0 > 0.0) {
      if (tol + 1.0 < 1.0) {
        y.set_size(1, 0);
      } else {
        y.set_size(1, static_cast<int>((tol + 1.0) - 1.0) + 1);
        indicesUpperBoundViolation_size = static_cast<int>((tol + 1.0) - 1.0);
        for (i = 0; i <= indicesUpperBoundViolation_size; i++) {
          y[i] = static_cast<double>(i) + 1.0;
        }
      }
      indicesUpperBoundViolation_size = y.size(1);
      r.set_size(1, y.size(1));
      for (i = 0; i < indicesUpperBoundViolation_size; i++) {
        r[i] = static_cast<int>(err + y[i]);
      }
      errPrev = positionMap_data[b_i];
      if (std::isnan(errPrev) || std::isnan(iter)) {
        y.set_size(1, 1);
        y[0] = rtNaN;
      } else if (iter < errPrev) {
        y.set_size(1, 0);
      } else if ((std::isinf(errPrev) || std::isinf(iter)) &&
                 (errPrev == iter)) {
        y.set_size(1, 1);
        y[0] = rtNaN;
      } else if (std::floor(errPrev) == errPrev) {
        indicesUpperBoundViolation_size = static_cast<int>(tol);
        y.set_size(1, static_cast<int>(tol) + 1);
        for (i = 0; i <= indicesUpperBoundViolation_size; i++) {
          y[i] = errPrev + static_cast<double>(i);
        }
      } else {
        ndbl = std::floor(tol + 0.5);
        apnd = errPrev + ndbl;
        cdiff = apnd - iter;
        if (std::abs(cdiff) <
            4.4408920985006262E-16 *
                std::fmax(std::abs(errPrev), std::abs(iter))) {
          ndbl++;
          apnd = iter;
        } else if (cdiff > 0.0) {
          apnd = errPrev + (ndbl - 1.0);
        } else {
          ndbl++;
        }
        if (ndbl >= 0.0) {
          indicesUpperBoundViolation_size = static_cast<int>(ndbl);
        } else {
          indicesUpperBoundViolation_size = 0;
        }
        y.set_size(1, indicesUpperBoundViolation_size);
        if (indicesUpperBoundViolation_size > 0) {
          y[0] = errPrev;
          if (indicesUpperBoundViolation_size > 1) {
            y[indicesUpperBoundViolation_size - 1] = apnd;
            nm1d2 = (indicesUpperBoundViolation_size - 1) / 2;
            for (k = 0; k <= nm1d2 - 2; k++) {
              y[k + 1] = errPrev + (static_cast<double>(k) + 1.0);
              y[(indicesUpperBoundViolation_size - k) - 2] =
                  apnd - (static_cast<double>(k) + 1.0);
            }
            if (nm1d2 << 1 == indicesUpperBoundViolation_size - 1) {
              y[nm1d2] = (positionMap_data[b_i] + apnd) / 2.0;
            } else {
              y[nm1d2] = positionMap_data[b_i] + static_cast<double>(nm1d2);
              y[nm1d2 + 1] = apnd - static_cast<double>(nm1d2);
            }
          }
        }
      }
      indicesUpperBoundViolation_size = y.size(1) - 1;
      for (i = 0; i <= indicesUpperBoundViolation_size; i++) {
        positionIndices[r[i] - 1] = y[i];
      }
      err += tol + 1.0;
    }
  }
  if (err < 1.0) {
    indicesUpperBoundViolation_size = 0;
  } else {
    indicesUpperBoundViolation_size = static_cast<int>(err);
  }
  positionIndices.set_size(positionIndices.size(0),
                           indicesUpperBoundViolation_size);
  for (i = 0; i < indicesUpperBoundViolation_size; i++) {
    err = positionIndices[i];
    varargin_1[static_cast<int>(err) - 1] = xSolPrev[static_cast<int>(err) - 1];
  }
  varargout_2->Iterations = iterations;
  varargout_2->NumRandomRestarts = rrAttempts;
  varargout_2->ExitFlag = static_cast<double>(exitFlagPrev);
}

} // namespace coder
} // namespace gik9dof

//
// File trailer for generalizedInverseKinematics.cpp
//
// [EOF]
//
