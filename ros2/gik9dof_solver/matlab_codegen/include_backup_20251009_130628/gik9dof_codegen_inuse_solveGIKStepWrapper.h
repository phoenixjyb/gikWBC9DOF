//
// File: gik9dof_codegen_inuse_solveGIKStepWrapper.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

#ifndef GIK9DOF_CODEGEN_INUSE_SOLVEGIKSTEPWRAPPER_H
#define GIK9DOF_CODEGEN_INUSE_SOLVEGIKSTEPWRAPPER_H

// Include Files
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace gik9dof {
namespace codegen_inuse {
extern void solveGIKStepWrapper(
    const double qCurrent[9], const double targetPose[16],
    const int distBodyIndices[20], const int distRefBodyIndices[20],
    const double distBoundsLower[20], const double distBoundsUpper[20],
    const double distWeights[20], double qNext[9], struct0_T *solverInfo);

}
} // namespace gik9dof
void solveGIKStepWrapper_delete();

void solveGIKStepWrapper_init();

void solveGIKStepWrapper_new();

#endif
//
// File trailer for gik9dof_codegen_inuse_solveGIKStepWrapper.h
//
// [EOF]
//
