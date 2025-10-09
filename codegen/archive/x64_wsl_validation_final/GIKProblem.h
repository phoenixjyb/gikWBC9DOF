//
// File: GIKProblem.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

#ifndef GIKPROBLEM_H
#define GIKPROBLEM_H

// Include Files
#include "DistanceBoundsConstraint.h"
#include "JointPositionBounds.h"
#include "PoseTarget.h"
#include "rtwtypes.h"
#include "solveGIKStepWrapper_types1.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
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
struct struct1_T;

} // namespace gik9dof

// Type Definitions
namespace gik9dof {
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class GIKProblem {
public:
  GIKProblem *init(b_RigidBodyTree *tree);
  void set_DesignVariableBounds(const ::coder::array<double, 2U> &b_value);
  void set_EnforceJointLimits(boolean_T b_value);
  void update(const constraintPoseTarget &varargin_1,
              const constraintJointBounds &varargin_2,
              const constraintDistanceBounds *varargin_3,
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
              const constraintDistanceBounds *varargin_22);
  void residuals(const ::coder::array<double, 1U> &x,
                 ::coder::array<double, 1U> &f);
  void residuals(const ::coder::array<double, 1U> &x,
                 ::coder::array<double, 1U> &f, ::coder::array<double, 2U> &J);
  void get_WeightMatrix(::coder::array<double, 2U> &b_value) const;
  double evaluateSolution(const ::coder::array<double, 1U> &x,
                          ::coder::array<double, 1U> &ev);
  void get_KinematicPath(double value_data[], int value_size[2]);
  void constraintViolations(const ::coder::array<double, 1U> &x,
                            struct1_T violations[22]);
  void b_set_EnforceJointLimits();
  GIKProblem();
  ~GIKProblem();

private:
  void updateDesignVariableBounds();
  void residualsInternal(const ::coder::array<double, 1U> &x,
                         ::coder::array<double, 1U> &f,
                         ::coder::array<double, 2U> &J);

public:
  boolean_T matlabCodegenIsDeleted;
  ::coder::array<double, 2U> DesignVariableBoundsInternal;
  b_RigidBodyTree *Tree;
  cell_6 Constraints;
  double NumResiduals;
  double NumSlacks;
  double NumPositions;
  double NumVariables;
  cell_wrap_7 ResidualIndices[22];
  cell_wrap_7 SlackIndices[22];
  DistanceBoundsConstraint _pobj0[20];
  JointPositionBounds _pobj1;
  PoseTarget _pobj2;

private:
  ::coder::array<double, 2U> ConstraintMatrixInternal;
  ::coder::array<double, 1U> ConstraintBoundInternal;
  boolean_T EnforceJointLimitsInternal;
  cell_wrap_8 EqualityFlags[22];
  ::coder::array<double, 1U> LastX;
  ::coder::array<double, 1U> LastF;
  ::coder::array<double, 2U> LastJ;
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
