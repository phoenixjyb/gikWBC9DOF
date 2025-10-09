//
// File: gik9dof_codegen_inuse_solveGIKStepWrapper_internal_types.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

#ifndef GIK9DOF_CODEGEN_INUSE_SOLVEGIKSTEPWRAPPER_INTERNAL_TYPES_H
#define GIK9DOF_CODEGEN_INUSE_SOLVEGIKSTEPWRAPPER_INTERNAL_TYPES_H

// Include Files
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "rtwtypes.h"

// Type Declarations
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class PoseTarget;

class JointPositionBounds;

class DistanceBoundsConstraint;

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

// Type Definitions
namespace coder {
namespace robotics {
namespace core {
namespace internal {
enum class NLPSolverExitFlags : int
{
  LocalMinimumFound = 1, // Default value
  IterationLimitExceeded,
  TimeLimitExceeded,
  StepSizeBelowMinimum,
  ChangeInErrorBelowMinimum,
  SearchDirectionInvalid,
  HessianNotPositiveSemidefinite,
  TrustRegionRadiusBelowMinimum
};

}
} // namespace core
} // namespace robotics
} // namespace coder
struct cell_7 {
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

#endif
//
// File trailer for gik9dof_codegen_inuse_solveGIKStepWrapper_internal_types.h
//
// [EOF]
//
