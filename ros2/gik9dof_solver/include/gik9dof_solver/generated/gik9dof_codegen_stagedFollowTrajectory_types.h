//
// gik9dof_codegen_stagedFollowTrajectory_types.h
//
// Code generation for function 'gik9dof.codegen.stagedFollowTrajectory'
//

#ifndef GIK9DOF_CODEGEN_STAGEDFOLLOWTRAJECTORY_TYPES_H
#define GIK9DOF_CODEGEN_STAGEDFOLLOWTRAJECTORY_TYPES_H

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
struct struct0_T {
  int stageA_samples;
  double rateHz;
  double hybridResolution;
  double hybridSafetyMargin;
  double hybridMinTurningRadius;
  double hybridMotionPrimitiveLength;
  double hybridHeadingBins;
  double hybridRotationStep;
  double maxLinearSpeed;
  double maxYawRate;
  double maxPathLength;
  double ppLookahead;
  double ppDesiredSpeed;
  double ppGoalRadius;
  double ppCloseOnHeading;
  double ppReverseAllowed;
};

namespace coder {
namespace robotics {
namespace core {
namespace internal {
#ifndef CODER_ROBOTICS_CORE_INTERNAL_NLPSOLVEREXITFLAGS
#define CODER_ROBOTICS_CORE_INTERNAL_NLPSOLVEREXITFLAGS
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
#endif // CODER_ROBOTICS_CORE_INTERNAL_NLPSOLVEREXITFLAGS

}
} // namespace core
} // namespace robotics
} // namespace coder
struct cell_wrap_33 {
  coder::array<double, 2U> f1;
};

struct cell_wrap_34 {
  coder::array<bool, 2U> f1;
};

struct cell_38 {
  coder::array<double, 2U> f1;
  coder::array<double, 2U> f2;
};

#ifndef GIK9DOF_STRUCT_T
#define GIK9DOF_STRUCT_T
struct struct_T {
  coder::bounded_array<char, 8U, 2U> Type;
  coder::array<double, 2U> Violation;
};
#endif // GIK9DOF_STRUCT_T

struct cell_36 {
  double f2[16];
  double f3;
  double f4;
  double f5[2];
};

struct cell_40 {
  double f2[2];
  double f3;
};

struct cell_32 {
  coder::robotics::manip::internal::PoseTarget *f1;
  coder::robotics::manip::internal::JointPositionBounds *f2;
  coder::robotics::manip::internal::DistanceBoundsConstraint *f3;
};

#endif
// End of code generation (gik9dof_codegen_stagedFollowTrajectory_types.h)
