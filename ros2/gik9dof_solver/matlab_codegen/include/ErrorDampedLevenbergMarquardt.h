//
// File: ErrorDampedLevenbergMarquardt.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

#ifndef ERRORDAMPEDLEVENBERGMARQUARDT_H
#define ERRORDAMPEDLEVENBERGMARQUARDT_H

// Include Files
#include "SystemTimeProvider.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_internal_types.h"
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
  double getSolverParams(
      char params_Name[18], double &params_MaxTime,
      double &params_GradientTolerance, double &params_SolutionTolerance,
      boolean_T &params_ConstraintsOn, boolean_T &params_RandomRestart,
      double &params_StepTolerance, double &params_ErrorChangeTolerance,
      double &params_DampingBias, boolean_T &params_UseErrorDamping) const;
  NLPSolverExitFlags solveInternal(array<double, 1U> &xSol, double &en,
                                   double &iter);
  boolean_T matlabCodegenIsDeleted;
  char Name[18];
  boolean_T ConstraintsOn;
  double SolutionTolerance;
  boolean_T RandomRestart;
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
  boolean_T UseErrorDamping;
  SystemTimeProvider TimeObjInternal;
};

} // namespace internal
} // namespace core
} // namespace robotics
} // namespace coder

#endif
//
// File trailer for ErrorDampedLevenbergMarquardt.h
//
// [EOF]
//
