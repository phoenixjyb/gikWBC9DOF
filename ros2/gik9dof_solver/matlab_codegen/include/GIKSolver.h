//
// File: GIKSolver.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

#ifndef GIKSOLVER_H
#define GIKSOLVER_H

// Include Files
#include "gik9dof_codegen_realtime_solveGIKStepWrapper_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace gik9dof {
class GIKSolver {
public:
  GIKSolver();
  ~GIKSolver();
  void gik9dof_codegen_realtime_solveGIKStepWrapper(const double qCurrent[9],
                                                    const double targetPose[16],
                                                    double distanceLower,
                                                    double distanceWeight,
                                                    double qNext[9],
                                                    struct0_T *solverInfo);
  void setMaxIterations(int max_iterations);
  gik9dof_codegen_realtime_solveGIKStepWrapperStackData *getStackData();

private:
  gik9dof_codegen_realtime_solveGIKStepWrapperPersistentData pd_;
  gik9dof_codegen_realtime_solveGIKStepWrapperStackData SD_;
  int max_iterations_{1500};  // Default max iterations, can be overridden
};

} // namespace gik9dof

#endif
//
// File trailer for GIKSolver.h
//
// [EOF]
//
