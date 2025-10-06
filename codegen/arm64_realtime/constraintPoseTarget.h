//
// File: constraintPoseTarget.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

#ifndef CONSTRAINTPOSETARGET_H
#define CONSTRAINTPOSETARGET_H

// Include Files
#include "gik9dof_codegen_realtime_solveGIKStepWrapper_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace gik9dof {
namespace coder {
class constraintPoseTarget {
public:
  constraintPoseTarget *init();
  constraintPoseTarget();
  ~constraintPoseTarget();
  char EndEffector[17];
  double TargetTransform[16];
  double OrientationTolerance;
  double PositionTolerance;
  double Weights[2];

protected:
  cell_47 ConstructorPropertyDefaultValues;
};

} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for constraintPoseTarget.h
//
// [EOF]
//
