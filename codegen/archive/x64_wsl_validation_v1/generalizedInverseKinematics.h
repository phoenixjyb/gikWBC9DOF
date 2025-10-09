//
// File: generalizedInverseKinematics.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 18:33:39
//

#ifndef GENERALIZEDINVERSEKINEMATICS_H
#define GENERALIZEDINVERSEKINEMATICS_H

// Include Files
#include "CollisionSet.h"
#include "ErrorDampedLevenbergMarquardt.h"
#include "GIKProblem.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "rigidBodyJoint.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
class GIKSolver;

namespace coder {
class rigidBodyTree;

class constraintPoseTarget;

class constraintJointBounds;

class constraintDistanceBounds;

} // namespace coder
struct struct0_T;

} // namespace gik9dof

// Type Definitions
namespace gik9dof {
namespace coder {
class generalizedInverseKinematics {
public:
  generalizedInverseKinematics *init(GIKSolver *aInstancePtr,
                                     rigidBodyTree &varargin_2);
  void set_SolverParameters(double solverparams_MaxIterations,
                            double solverparams_MaxTime,
                            double solverparams_GradientTolerance,
                            double solverparams_SolutionTolerance,
                            bool solverparams_EnforceJointLimits,
                            bool solverparams_AllowRandomRestart,
                            double solverparams_StepTolerance,
                            double solverparams_ErrorChangeTolerance,
                            double solverparams_DampingBias,
                            bool solverparams_UseErrorDamping);
  void matlabCodegenDestructor();
  void step(GIKSolver *aInstancePtr, double varargin_1[9],
            const constraintPoseTarget &varargin_2,
            const constraintJointBounds &varargin_3,
            constraintDistanceBounds *varargin_4,
            constraintDistanceBounds *varargin_5,
            constraintDistanceBounds *varargin_6,
            constraintDistanceBounds *varargin_7,
            constraintDistanceBounds *varargin_8,
            constraintDistanceBounds *varargin_9,
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
            constraintDistanceBounds *varargin_23, struct0_T *varargout_2);
  ~generalizedInverseKinematics();
  generalizedInverseKinematics();
  bool matlabCodegenIsDeleted;
  robotics::core::internal::ErrorDampedLevenbergMarquardt *Solver;
  robotics::manip::internal::b_RigidBodyTree *Tree;
  robotics::manip::internal::GIKProblem Problem;
  bool EnforceJointLimits;
  rigidBodyJoint _pobj0[22];
  robotics::manip::internal::RigidBody _pobj1[11];
  robotics::manip::internal::CollisionSet _pobj2[23];
  robotics::manip::internal::b_RigidBodyTree _pobj3;
  robotics::core::internal::ErrorDampedLevenbergMarquardt _pobj4;

private:
  int isInitialized;
  bool isSetupComplete;
  bool RigidBodyTreeHasBeenSet;
};

} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for generalizedInverseKinematics.h
//
// [EOF]
//
