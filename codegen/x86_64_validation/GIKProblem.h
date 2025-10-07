//
// File: GIKProblem.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

#ifndef GIKPROBLEM_H
#define GIKPROBLEM_H

// Include Files
#include "DistanceBoundsConstraint.h"
#include "JointPositionBounds.h"
#include "PoseTarget.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types1.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class RigidBodyTree;

}
} // namespace manip
} // namespace robotics
class constraintPoseTarget;

class constraintJointBounds;

class constraintDistanceBounds;

} // namespace coder
} // namespace gik9dof

// Type Definitions
namespace gik9dof {
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class GIKProblem {
public:
  void updateDesignVariableBounds();
  void residualsInternal(const ::coder::array<double, 1U> &x,
                         ::coder::array<double, 1U> &f,
                         ::coder::array<double, 2U> &J);
  void set_EnforceJointLimits(bool b_value);
  void update(const constraintPoseTarget &varargin_1,
              const constraintJointBounds &varargin_2,
              const constraintDistanceBounds &varargin_3);
  void residuals(const ::coder::array<double, 1U> &x,
                 ::coder::array<double, 1U> &f);
  void residuals(const ::coder::array<double, 1U> &x,
                 ::coder::array<double, 1U> &f, ::coder::array<double, 2U> &J);
  void get_WeightMatrix(::coder::array<double, 2U> &b_value) const;
  double evaluateSolution(const ::coder::array<double, 1U> &x,
                          ::coder::array<double, 1U> &ev);
  void get_KinematicPath(double value_data[], int value_size[2]);
  GIKProblem();
  ~GIKProblem();
  bool matlabCodegenIsDeleted;
  ::coder::array<double, 2U> DesignVariableBoundsInternal;
  ::coder::array<double, 2U> ConstraintMatrixInternal;
  ::coder::array<double, 1U> ConstraintBoundInternal;
  RigidBodyTree *Tree;
  cell_6 Constraints;
  double NumResiduals;
  double NumSlacks;
  double NumPositions;
  double NumVariables;
  cell_wrap_7 ResidualIndices[3];
  cell_wrap_7 SlackIndices[3];
  bool EnforceJointLimitsInternal;
  cell_wrap_8 EqualityFlags[3];
  ::coder::array<double, 1U> LastX;
  ::coder::array<double, 1U> LastF;
  ::coder::array<double, 2U> LastJ;
  DistanceBoundsConstraint _pobj0;
  JointPositionBounds _pobj1;
  PoseTarget _pobj2;
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for GIKProblem.h
//
// [EOF]
//
