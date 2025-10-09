//
// File: constraintPoseTarget.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

#ifndef CONSTRAINTPOSETARGET_H
#define CONSTRAINTPOSETARGET_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
struct cell_11 {
  double f2[16];
  double f3;
  double f4;
  double f5[2];
};

namespace coder {
class constraintPoseTarget {
public:
  constraintPoseTarget *init();
  char EndEffector[17];
  double TargetTransform[16];
  double OrientationTolerance;
  double PositionTolerance;
  double Weights[2];

protected:
  cell_11 ConstructorPropertyDefaultValues;
};

} // namespace coder

#endif
//
// File trailer for constraintPoseTarget.h
//
// [EOF]
//
