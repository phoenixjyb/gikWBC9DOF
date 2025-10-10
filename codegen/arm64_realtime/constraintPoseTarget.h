//
// File: constraintPoseTarget.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 19:17:46
//

#ifndef CONSTRAINTPOSETARGET_H
#define CONSTRAINTPOSETARGET_H

// Include Files
#include "rtwtypes.h"
#include "solveGIKStepWrapper_types1.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace gik9dof {
namespace coder {
class constraintPoseTarget {
public:
  constraintPoseTarget();
  ~constraintPoseTarget();
  cell_10 ConstructorPropertyDefaultValues;
  char EndEffector[17];
  double TargetTransform[16];
  double OrientationTolerance;
  double PositionTolerance;
  double Weights[2];
};

} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for constraintPoseTarget.h
//
// [EOF]
//
