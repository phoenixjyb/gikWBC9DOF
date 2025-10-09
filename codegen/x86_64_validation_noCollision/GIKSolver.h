//
// File: GIKSolver.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

#ifndef GIKSOLVER_H
#define GIKSOLVER_H

// Include Files
#include "rtwtypes.h"
#include "solveGIKStepWrapper_types.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace gik9dof {
class GIKSolver {
public:
  GIKSolver();
  ~GIKSolver();
  void solveGIKStepWrapper(
      const double qCurrent[9], const double targetPose[16],
      const int distBodyIndices[20], const int distRefBodyIndices[20],
      const double distBoundsLower[20], const double distBoundsUpper[20],
      const double distWeights[20], double qNext[9], struct0_T *solverInfo);
  solveGIKStepWrapperStackData *getStackData();

private:
  solveGIKStepWrapperPersistentData pd_;
  solveGIKStepWrapperStackData SD_;
};

} // namespace gik9dof

#endif
//
// File trailer for GIKSolver.h
//
// [EOF]
//
