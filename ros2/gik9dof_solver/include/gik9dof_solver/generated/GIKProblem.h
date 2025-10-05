//
// GIKProblem.h
//
// Code generation for function 'GIKProblem'
//

#ifndef GIKPROBLEM_H
#define GIKPROBLEM_H

// Include files
#include "DistanceBoundsConstraint.h"
#include "JointPositionBounds.h"
#include "PoseTarget.h"
#include "gik9dof_codegen_followTrajectory_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class b_RigidBodyTree;

}
} // namespace manip
} // namespace robotics
class constraintPoseTarget;

class constraintJointBounds;

class constraintDistanceBounds;

} // namespace coder

// Type Definitions
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class GIKProblem {
public:
  void updateDesignVariableBounds();
  void residualsInternal(const array<double, 1U> &x, array<double, 1U> &f,
                         array<double, 2U> &J);
  void set_EnforceJointLimits(bool b_value);
  void update(const constraintPoseTarget &varargin_1,
              const constraintJointBounds &varargin_2,
              const constraintDistanceBounds &varargin_3);
  void residuals(const array<double, 1U> &x, array<double, 1U> &f);
  void residuals(const array<double, 1U> &x, array<double, 1U> &f,
                 array<double, 2U> &J);
  void get_WeightMatrix(array<double, 2U> &b_value) const;
  double evaluateSolution(const array<double, 1U> &x, array<double, 1U> &ev);
  void get_KinematicPath(double value_data[], int value_size[2]);
  GIKProblem();
  ~GIKProblem();
  bool matlabCodegenIsDeleted;
  array<double, 2U> DesignVariableBoundsInternal;
  array<double, 2U> ConstraintMatrixInternal;
  array<double, 1U> ConstraintBoundInternal;
  b_RigidBodyTree *Tree;
  cell_6 Constraints;
  double NumResiduals;
  double NumSlacks;
  double NumPositions;
  double NumVariables;
  cell_wrap_7 ResidualIndices[3];
  cell_wrap_7 SlackIndices[3];
  bool EnforceJointLimitsInternal;
  cell_wrap_8 EqualityFlags[3];
  array<double, 1U> LastX;
  array<double, 1U> LastF;
  array<double, 2U> LastJ;
  DistanceBoundsConstraint _pobj0;
  JointPositionBounds _pobj1;
  PoseTarget _pobj2;
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

#endif
// End of code generation (GIKProblem.h)
