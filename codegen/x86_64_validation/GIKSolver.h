//
// File: GIKSolver.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 18:33:39
//

#ifndef GIKSOLVER_H
#define GIKSOLVER_H

// Include Files
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace gik9dof {
class GIKSolver {
public:
  GIKSolver();
  ~GIKSolver();
  void gik9dof_codegen_inuse_solveGIKStepWrapper(
      const double qCurrent[9], const double targetPose[16],
      const int distBodyIndices[20], const int distRefBodyIndices[20],
      const double distBoundsLower[20], const double distBoundsUpper[20],
      const double distWeights[20], double qNext[9], struct0_T *solverInfo);
  gik9dof_codegen_inuse_solveGIKStepWrapperStackData *getStackData();

private:
  gik9dof_codegen_inuse_solveGIKStepWrapperPersistentData pd_;
  gik9dof_codegen_inuse_solveGIKStepWrapperStackData SD_;
};

} // namespace gik9dof

#endif
//
// File trailer for GIKSolver.h
//
// [EOF]
//
