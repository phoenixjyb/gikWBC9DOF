//
// constraintPoseTarget.h
//
// Code generation for function 'constraintPoseTarget'
//

#ifndef CONSTRAINTPOSETARGET_H
#define CONSTRAINTPOSETARGET_H

// Include files
#include "gik9dof_codegen_followTrajectory_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
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
  cell_10 ConstructorPropertyDefaultValues;
};

} // namespace coder

#endif
// End of code generation (constraintPoseTarget.h)
