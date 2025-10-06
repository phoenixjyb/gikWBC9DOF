//
// File: solveGIKStepRealtime.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

#ifndef SOLVEGIKSTEPREALTIME_H
#define SOLVEGIKSTEPREALTIME_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
class GIKSolver;

namespace coder {
class rigidBodyTree;

class generalizedInverseKinematics;

} // namespace coder
struct struct1_T;

} // namespace gik9dof

// Function Declarations
namespace gik9dof {
namespace codegen_realtime {
double solveGIKStepRealtime(
    GIKSolver *aInstancePtr, coder::rigidBodyTree &robot,
    coder::generalizedInverseKinematics &solver, const double qCurrent[9],
    const double targetPose[16], double distanceLower, double distanceWeight,
    double qNext[9], struct1_T solverInfo_ConstraintViolations[3],
    char solverInfo_Status_data[], int solverInfo_Status_size[2],
    double &solverInfo_NumRandomRestarts, double &solverInfo_ExitFlag);

}
} // namespace gik9dof

#endif
//
// File trailer for solveGIKStepRealtime.h
//
// [EOF]
//
