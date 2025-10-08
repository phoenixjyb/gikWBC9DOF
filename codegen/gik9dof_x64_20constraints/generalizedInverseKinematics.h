//
// File: generalizedInverseKinematics.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
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
namespace coder {
class rigidBodyTree;

class constraintPoseTarget;

class constraintJointBounds;

class constraintDistanceBounds;

} // namespace coder
struct struct0_T;

// Type Definitions
namespace coder {
class generalizedInverseKinematics {
public:
  generalizedInverseKinematics *init(rigidBodyTree &varargin_2);
  void set_SolverParameters(double solverparams_MaxIterations,
                            double solverparams_MaxTime,
                            double solverparams_GradientTolerance,
                            double solverparams_SolutionTolerance,
                            boolean_T solverparams_EnforceJointLimits,
                            boolean_T solverparams_AllowRandomRestart,
                            double solverparams_StepTolerance,
                            double solverparams_ErrorChangeTolerance,
                            double solverparams_DampingBias,
                            boolean_T solverparams_UseErrorDamping);
  void matlabCodegenDestructor();
  void step(double varargin_1[9], const constraintPoseTarget &varargin_2,
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
  boolean_T matlabCodegenIsDeleted;
  robotics::core::internal::ErrorDampedLevenbergMarquardt *Solver;
  robotics::manip::internal::RigidBodyTree *Tree;
  robotics::manip::internal::GIKProblem Problem;
  boolean_T EnforceJointLimits;
  rigidBodyJoint _pobj0[22];
  robotics::manip::internal::RigidBody _pobj1[11];
  robotics::manip::internal::CollisionSet _pobj2[23];
  robotics::manip::internal::RigidBodyTree _pobj3;
  robotics::core::internal::ErrorDampedLevenbergMarquardt _pobj4;

private:
  int isInitialized;
  boolean_T isSetupComplete;
  boolean_T RigidBodyTreeHasBeenSet;
};

} // namespace coder

#endif
//
// File trailer for generalizedInverseKinematics.h
//
// [EOF]
//
