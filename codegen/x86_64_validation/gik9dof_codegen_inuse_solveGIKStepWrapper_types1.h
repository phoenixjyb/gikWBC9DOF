//
// File: gik9dof_codegen_inuse_solveGIKStepWrapper_types1.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

#ifndef GIK9DOF_CODEGEN_INUSE_SOLVEGIKSTEPWRAPPER_TYPES1_H
#define GIK9DOF_CODEGEN_INUSE_SOLVEGIKSTEPWRAPPER_TYPES1_H

// Include Files
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
class PoseTarget;

class JointPositionBounds;

class DistanceBoundsConstraint;

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

// Type Definitions
namespace gik9dof {
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
struct cell_wrap_7 {
  ::coder::array<double, 2U> f1;
};

struct cell_wrap_8 {
  ::coder::array<bool, 2U> f1;
};

struct cell_6 {
  coder::robotics::manip::internal::PoseTarget *f1;
  coder::robotics::manip::internal::JointPositionBounds *f2;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f3;
};

} // namespace gik9dof

#endif
//
// File trailer for gik9dof_codegen_inuse_solveGIKStepWrapper_types1.h
//
// [EOF]
//
