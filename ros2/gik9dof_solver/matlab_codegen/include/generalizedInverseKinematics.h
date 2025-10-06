//
// File: generalizedInverseKinematics.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
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
struct struct1_T;

} // namespace gik9dof

// Type Definitions
namespace gik9dof {
namespace coder {
class generalizedInverseKinematics {
public:
  generalizedInverseKinematics *init(GIKSolver *aInstancePtr,
                                     rigidBodyTree &varargin_2);
  double step(GIKSolver *aInstancePtr, const double varargin_1[9],
              constraintPoseTarget &varargin_2,
              constraintJointBounds &varargin_3,
              constraintDistanceBounds &varargin_4, double varargout_1[9],
              struct1_T varargout_2_ConstraintViolations[3],
              char varargout_2_Status_data[], int varargout_2_Status_size[2],
              double &varargout_2_NumRandomRestarts,
              double &varargout_2_ExitFlag);
  void matlabCodegenDestructor();
  ~generalizedInverseKinematics();
  generalizedInverseKinematics();

protected:
  double stepImpl(GIKSolver *aInstancePtr, double initialGuess[9],
                  const constraintPoseTarget &varargin_1,
                  const constraintJointBounds &varargin_2,
                  const constraintDistanceBounds &varargin_3,
                  struct1_T solutionInfo_ConstraintViolations[3],
                  char solutionInfo_Status_data[],
                  int solutionInfo_Status_size[2],
                  double &solutionInfo_NumRandomRestarts,
                  double &solutionInfo_ExitFlag);

public:
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
} // namespace gik9dof

#endif
//
// File trailer for generalizedInverseKinematics.h
//
// [EOF]
//
