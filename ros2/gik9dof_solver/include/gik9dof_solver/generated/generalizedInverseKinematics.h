//
// generalizedInverseKinematics.h
//
// Code generation for function 'generalizedInverseKinematics'
//

#ifndef GENERALIZEDINVERSEKINEMATICS_H
#define GENERALIZEDINVERSEKINEMATICS_H

// Include files
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
struct struct_T;

// Type Definitions
namespace coder {
class generalizedInverseKinematics {
public:
  generalizedInverseKinematics *init(rigidBodyTree &varargin_2);
  void step(const double varargin_1[9], constraintPoseTarget &varargin_2,
            constraintJointBounds &varargin_3,
            constraintDistanceBounds &varargin_4, double varargout_1[9]);
  void matlabCodegenDestructor();
  ~generalizedInverseKinematics();
  generalizedInverseKinematics();

protected:
  double stepImpl(double initialGuess[9],
                  const constraintPoseTarget &varargin_1,
                  const constraintJointBounds &varargin_2,
                  const constraintDistanceBounds &varargin_3,
                  struct_T solutionInfo_ConstraintViolations[3],
                  char solutionInfo_Status_data[],
                  int solutionInfo_Status_size[2],
                  double &solutionInfo_NumRandomRestarts,
                  double &solutionInfo_ExitFlag);

public:
  bool matlabCodegenIsDeleted;
  robotics::core::internal::ErrorDampedLevenbergMarquardt *Solver;
  robotics::manip::internal::b_RigidBodyTree *Tree;
  robotics::manip::internal::GIKProblem Problem;
  bool EnforceJointLimits;
  rigidBodyJoint _pobj0[24];
  robotics::manip::internal::RigidBody _pobj1[12];
  robotics::manip::internal::CollisionSet _pobj2[25];
  robotics::manip::internal::b_RigidBodyTree _pobj3;
  robotics::core::internal::ErrorDampedLevenbergMarquardt _pobj4;

private:
  int isInitialized;
  bool isSetupComplete;
  bool RigidBodyTreeHasBeenSet;
};

} // namespace coder

#endif
// End of code generation (generalizedInverseKinematics.h)
