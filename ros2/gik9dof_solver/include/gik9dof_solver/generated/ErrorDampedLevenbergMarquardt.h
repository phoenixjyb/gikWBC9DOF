//
// ErrorDampedLevenbergMarquardt.h
//
// Code generation for function 'ErrorDampedLevenbergMarquardt'
//

#ifndef ERRORDAMPEDLEVENBERGMARQUARDT_H
#define ERRORDAMPEDLEVENBERGMARQUARDT_H

// Include files
#include "SystemTimeProvider.h"
#include "gik9dof_codegen_followTrajectory_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class GIKProblem;

}
} // namespace manip
} // namespace robotics
} // namespace coder

// Type Definitions
namespace coder {
namespace robotics {
namespace core {
namespace internal {
class ErrorDampedLevenbergMarquardt {
public:
  NLPSolverExitFlags solveInternal(array<double, 1U> &xSol, double &en,
                                   double &iter);
  ErrorDampedLevenbergMarquardt();
  ~ErrorDampedLevenbergMarquardt();
  bool matlabCodegenIsDeleted;
  char Name[18];
  bool ConstraintsOn;
  double SolutionTolerance;
  bool RandomRestart;
  manip::internal::GIKProblem *ExtraArgs;
  double MaxNumIteration;
  double MaxTime;
  array<double, 1U> SeedInternal;
  double MaxTimeInternal;
  double MaxNumIterationInternal;
  double StepTolerance;
  SystemTimeProvider TimeObj;
  double GradientTolerance;
  double ErrorChangeTolerance;
  double DampingBias;
  bool UseErrorDamping;
  SystemTimeProvider TimeObjInternal;
};

} // namespace internal
} // namespace core
} // namespace robotics
} // namespace coder

#endif
// End of code generation (ErrorDampedLevenbergMarquardt.h)
