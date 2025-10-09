//
// File: gik9dof_codegen_inuse_solveGIKStepWrapper_internal_types.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
//

#ifndef GIK9DOF_CODEGEN_INUSE_SOLVEGIKSTEPWRAPPER_INTERNAL_TYPES_H
#define GIK9DOF_CODEGEN_INUSE_SOLVEGIKSTEPWRAPPER_INTERNAL_TYPES_H

// Include Files
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "rtwtypes.h"

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

#endif
//
// File trailer for gik9dof_codegen_inuse_solveGIKStepWrapper_internal_types.h
//
// [EOF]
//
