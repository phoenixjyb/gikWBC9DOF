//
// gik9dof_codegen_followTrajectory_types.h
//
// Code generation for function 'gik9dof.codegen.followTrajectory'
//

#ifndef GIK9DOF_CODEGEN_FOLLOWTRAJECTORY_TYPES_H
#define GIK9DOF_CODEGEN_FOLLOWTRAJECTORY_TYPES_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include "coder_bounded_array.h"
#define MAX_THREADS omp_get_max_threads()

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
struct cell_wrap_7 {
  coder::array<double, 2U> f1;
};

struct cell_wrap_8 {
  coder::array<bool, 2U> f1;
};

struct cell_12 {
  coder::array<double, 2U> f1;
  coder::array<double, 2U> f2;
};

struct struct_T {
  coder::bounded_array<char, 8U, 2U> Type;
  coder::array<double, 2U> Violation;
};

struct cell_10 {
  double f2[16];
  double f3;
  double f4;
  double f5[2];
};

struct cell_14 {
  double f2[2];
  double f3;
};

struct cell_6 {
  coder::robotics::manip::internal::PoseTarget *f1;
  coder::robotics::manip::internal::JointPositionBounds *f2;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f3;
};

#endif
// End of code generation (gik9dof_codegen_followTrajectory_types.h)
