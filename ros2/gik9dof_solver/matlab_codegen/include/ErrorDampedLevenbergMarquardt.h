//
// File: ErrorDampedLevenbergMarquardt.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

#ifndef ERRORDAMPEDLEVENBERGMARQUARDT_H
#define ERRORDAMPEDLEVENBERGMARQUARDT_H

// Include Files
#include "SystemTimeProvider.h"
#include "gik9dof_codegen_realtime_solveGIKStepWrapper_types1.h"
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
class GIKProblem;

}
} // namespace manip
} // namespace robotics
} // namespace coder
class GIKSolver;

} // namespace gik9dof

// Type Definitions
namespace gik9dof {
namespace coder {
namespace robotics {
namespace core {
namespace internal {
class ErrorDampedLevenbergMarquardt {
public:
  NLPSolverExitFlags solveInternal(GIKSolver *aInstancePtr,
                                   ::coder::array<double, 1U> &xSol, double &en,
                                   double &iter);
  ErrorDampedLevenbergMarquardt();
  ~ErrorDampedLevenbergMarquardt();
  boolean_T matlabCodegenIsDeleted;
  char Name[18];
  boolean_T ConstraintsOn;
  double SolutionTolerance;
  boolean_T RandomRestart;
  manip::internal::GIKProblem *ExtraArgs;
  double MaxNumIteration;
  double MaxTime;
  ::coder::array<double, 1U> SeedInternal;
  double MaxTimeInternal;
  double MaxNumIterationInternal;
  double StepTolerance;
  SystemTimeProvider TimeObj;
  double GradientTolerance;
  double ErrorChangeTolerance;
  double DampingBias;
  boolean_T UseErrorDamping;
  SystemTimeProvider TimeObjInternal;
};

} // namespace internal
} // namespace core
} // namespace robotics
} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for ErrorDampedLevenbergMarquardt.h
//
// [EOF]
//
