//
// File: GIKProblem.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

#ifndef GIKPROBLEM_H
#define GIKPROBLEM_H

// Include Files
#include "DistanceBoundsConstraint.h"
#include "JointPositionBounds.h"
#include "PoseTarget.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_internal_types.h"
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
} // namespace coder
struct struct1_T;

// Type Definitions
struct cell_wrap_8 {
  coder::array<double, 2U> f1;
};

struct cell_wrap_9 {
  coder::array<boolean_T, 2U> f1;
};

namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class GIKProblem {
public:
  GIKProblem *init(RigidBodyTree *tree);
  void set_DesignVariableBounds(const array<double, 2U> &b_value);
  void updateDesignVariableBounds();
  void residualsInternal(const array<double, 1U> &x, array<double, 1U> &f,
                         array<double, 2U> &J);
  void set_EnforceJointLimits(boolean_T b_value);
  void residuals(const array<double, 1U> &x, array<double, 1U> &f);
  void residuals(const array<double, 1U> &x, array<double, 1U> &f,
                 array<double, 2U> &J);
  void get_WeightMatrix(array<double, 2U> &b_value) const;
  double evaluateSolution(const array<double, 1U> &x, array<double, 1U> &ev);
  void get_KinematicPath(double value_data[], int value_size[2]);
  void constraintViolations(const array<double, 1U> &x,
                            struct1_T violations[22]);
  void b_set_EnforceJointLimits();
  boolean_T matlabCodegenIsDeleted;
  array<double, 2U> DesignVariableBoundsInternal;
  RigidBodyTree *Tree;
  cell_7 Constraints;
  double NumResiduals;
  double NumSlacks;
  double NumPositions;
  double NumVariables;
  cell_wrap_8 ResidualIndices[22];
  cell_wrap_8 SlackIndices[22];
  array<double, 1U> LastX;
  array<double, 1U> LastF;
  array<double, 2U> LastJ;
  DistanceBoundsConstraint _pobj0[20];
  JointPositionBounds _pobj1;
  PoseTarget _pobj2;

private:
  array<double, 2U> ConstraintMatrixInternal;
  array<double, 1U> ConstraintBoundInternal;
  boolean_T EnforceJointLimitsInternal;
  cell_wrap_9 EqualityFlags[22];
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
