//
// File: ErrorDampedLevenbergMarquardt.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 09-Oct-2025 12:02:50
//

#ifndef ERRORDAMPEDLEVENBERGMARQUARDT_H
#define ERRORDAMPEDLEVENBERGMARQUARDT_H

// Include Files
#include "SystemTimeProvider.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types1.h"
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
  double getSolverParams(char params_Name[18], double &params_MaxTime,
                         double &params_GradientTolerance,
                         double &params_SolutionTolerance,
                         bool &params_ConstraintsOn, bool &params_RandomRestart,
                         double &params_StepTolerance,
                         double &params_ErrorChangeTolerance,
                         double &params_DampingBias,
                         bool &params_UseErrorDamping) const;
  NLPSolverExitFlags solveInternal(GIKSolver *aInstancePtr,
                                   ::coder::array<double, 1U> &xSol, double &en,
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
  ::coder::array<double, 1U> SeedInternal;
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
} // namespace gik9dof

#endif
//
// File trailer for ErrorDampedLevenbergMarquardt.h
//
// [EOF]
//
