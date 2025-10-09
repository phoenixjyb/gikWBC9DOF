//
// File: GIKProblem.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
//

#ifndef GIKPROBLEM_H
#define GIKPROBLEM_H

// Include Files
#include "DistanceBoundsConstraint.h"
#include "JointPositionBounds.h"
#include "PoseTarget.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
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
struct struct1_T;

// Type Definitions
struct cell_wrap_7 {
  coder::array<double, 2U> f1;
};

struct cell_wrap_8 {
  coder::array<boolean_T, 2U> f1;
};

struct cell_6 {
  coder::robotics::manip::internal::PoseTarget *f1;
  coder::robotics::manip::internal::JointPositionBounds *f2;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f3;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f4;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f5;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f6;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f7;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f8;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f9;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f10;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f11;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f12;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f13;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f14;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f15;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f16;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f17;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f18;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f19;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f20;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f21;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f22;
};

namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class GIKProblem {
public:
  GIKProblem *init(RigidBodyTree *tree);
  void set_DesignVariableBounds(const array<double, 2U> &b_value);
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
  void residuals(const array<double, 1U> &x, array<double, 1U> &f);
  void residuals(const array<double, 1U> &x, array<double, 1U> &f,
                 array<double, 2U> &J);
  void get_WeightMatrix(array<double, 2U> &b_value) const;
  double evaluateSolution(const array<double, 1U> &x, array<double, 1U> &ev);
  void get_KinematicPath(double value_data[], int value_size[2]);
  void constraintViolations(const array<double, 1U> &x,
                            struct1_T violations[22]);
  void b_set_EnforceJointLimits();

private:
  void updateDesignVariableBounds();
  void residualsInternal(const array<double, 1U> &x, array<double, 1U> &f,
                         array<double, 2U> &J);

public:
  boolean_T matlabCodegenIsDeleted;
  array<double, 2U> DesignVariableBoundsInternal;
  RigidBodyTree *Tree;
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
  array<double, 2U> ConstraintMatrixInternal;
  array<double, 1U> ConstraintBoundInternal;
  boolean_T EnforceJointLimitsInternal;
  cell_wrap_8 EqualityFlags[22];
  array<double, 1U> LastX;
  array<double, 1U> LastF;
  array<double, 2U> LastJ;
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

#endif
//
// File trailer for GIKProblem.h
//
// [EOF]
//
