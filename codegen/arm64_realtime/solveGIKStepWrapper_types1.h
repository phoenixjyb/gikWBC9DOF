//
// File: solveGIKStepWrapper_types1.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 19:17:46
//

#ifndef SOLVEGIKSTEPWRAPPER_TYPES1_H
#define SOLVEGIKSTEPWRAPPER_TYPES1_H

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

struct cell_12 {
  ::coder::array<double, 2U> f1;
  ::coder::array<double, 2U> f2;
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

} // namespace gik9dof

#endif
//
// File trailer for solveGIKStepWrapper_types1.h
//
// [EOF]
//
